import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
import rasterio
from rasterio.transform import from_origin

spec = importlib.util.spec_from_file_location("terrain", Path(__file__).resolve().parents[1] / "map/build_terrain.py")
terrain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terrain)


class TerrainTests(unittest.TestCase):
    def test_grid_alignment_and_mesh(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            tile = base / "plane.tif"
            # Pixel centres span E=0..10, N=10..0. Height = E + 2*N.
            e, n = np.meshgrid(np.arange(11), np.arange(10, -1, -1))
            heights = (e + 2 * n).astype(np.float32)
            with rasterio.open(tile, "w", driver="GTiff", width=11, height=11,
                               count=1, dtype="float32", crs="EPSG:25832",
                               transform=from_origin(-0.5, 10.5, 1, 1), nodata=-9999) as out:
                out.write(heights, 1)
            sampled, filled = terrain.sample_grid([tile], [2, 2, 8, 8], 2)
            self.assertEqual(sampled.shape, (4, 4))
            self.assertEqual(filled, 0)
            self.assertAlmostEqual(float(sampled[0, 0]), 18)
            self.assertAlmostEqual(float(sampled[-1, -1]), 12)
            metadata = dict(origin_easting=5, origin_northing=5, origin_height=1)
            obj = base / "terrain.obj"
            terrain.write_mesh(obj, metadata, [2, 2, 8, 8], sampled, 2, True)
            lines = obj.read_text().splitlines()
            self.assertIn("v -3.000000 17.000000 3.000000", lines)
            self.assertIn("vt 0.00000000 0.00000000", lines)
            self.assertEqual(sum(line.startswith("f ") for line in lines), 18)
            self.assertIn("map_Kd terrain_map.png", obj.with_suffix(".mtl").read_text())
            with self.assertRaises(ValueError):
                terrain.sample_grid([tile], [-10, -10, 10, 10], 2)


if __name__ == "__main__":
    unittest.main()
