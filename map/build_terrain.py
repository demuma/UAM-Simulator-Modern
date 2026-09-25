"""Build aligned terrain OBJ/MTL from EPSG:25832 DGM GeoTIFF tiles."""
import argparse
from contextlib import ExitStack
import json
import math
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen

import numpy as np
import rasterio
from rasterio.transform import from_origin
from rasterio.warp import reproject, Resampling
from rasterio.fill import fillnodata


def sample_grid(paths, bounds, spacing, fill_gaps_metres=0):
    west, south, east, north = bounds
    columns = round((east - west) / spacing) + 1
    rows = round((north - south) / spacing) + 1
    if columns < 2 or rows < 2 or columns * rows > 4_000_000:
        raise ValueError("Terrain grid must contain 4..4,000,000 vertices")
    heights = np.full((rows, columns), np.nan, dtype=np.float32)
    transform = from_origin(west - spacing / 2, north + spacing / 2, spacing, spacing)
    with ExitStack() as stack:
        for path in paths:
            dataset = stack.enter_context(rasterio.open(path))
            if dataset.crs is None or dataset.crs.to_epsg() != 25832:
                raise ValueError("Expected EPSG:25832: {}".format(path))
            tile = np.full_like(heights, np.nan)
            reproject(rasterio.band(dataset, 1), tile,
                      dst_transform=transform, dst_crs="EPSG:25832", dst_nodata=np.nan,
                      resampling=Resampling.bilinear)
            tile = tile * dataset.scales[0] + dataset.offsets[0]
            valid = np.isfinite(tile)
            heights[valid] = tile[valid]
    missing = int(np.count_nonzero(~np.isfinite(heights)))
    if missing and fill_gaps_metres > 0:
        valid = np.isfinite(heights)
        heights = fillnodata(np.where(valid, heights, -99999).astype(np.float32),
                            mask=valid.astype(np.uint8), max_search_distance=fill_gaps_metres / spacing)
        heights[heights == -99999] = np.nan
        print("Interpolated", missing - int(np.count_nonzero(~np.isfinite(heights))), "missing vertices within", fill_gaps_metres, "m")
    remaining = int(np.count_nonzero(~np.isfinite(heights)))
    if remaining:
        raise ValueError("{} grid vertices lack DGM data; supply all covering tiles or explicitly allow small-gap interpolation".format(remaining))
    return heights, missing


def write_mesh(output, metadata, bounds, heights, spacing, textured):
    west, south, east, north = bounds
    rows, columns = heights.shape
    gradient_south, gradient_east = np.gradient(heights, spacing)
    with output.open("w") as obj:
        obj.write("# DGM terrain, OBJ axes east/up/north; renderer mirrors X\nmtllib terrain.mtl\nusemtl terrain\n")
        for row in range(rows):
            for column in range(columns):
                easting, northing = west + column * spacing, north - row * spacing
                obj.write("v {:.6f} {:.6f} {:.6f}\n".format(
                    easting - metadata["origin_easting"],
                    float(heights[row, column]) - metadata["origin_height"],
                    northing - metadata["origin_northing"]))
                obj.write("vt {:.8f} {:.8f}\n".format(column / (columns - 1), row / (rows - 1)))
                normal = np.array([-gradient_east[row, column], 1.0, gradient_south[row, column]])
                normal /= np.linalg.norm(normal)
                obj.write("vn {:.7f} {:.7f} {:.7f}\n".format(*normal))
        for row in range(rows - 1):
            for column in range(columns - 1):
                a = row * columns + column + 1
                b, c, d = a + 1, a + columns, a + columns + 1
                for triangle in ((a, b, c), (b, d, c)):
                    obj.write("f " + " ".join("{0}/{0}/{0}".format(i) for i in triangle) + "\n")
    output.with_suffix(".mtl").write_text(
        "newmtl terrain\n" + ("Kd 1 1 1\nmap_Kd terrain_map.png\n" if textured else "Kd 0.35 0.43 0.32\n"))


def main():
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tiles", nargs="+", type=Path)
    parser.add_argument("--georef", type=Path, default=base / "hh_clip.georef.json")
    parser.add_argument("--output-dir", type=Path, default=base)
    parser.add_argument("--spacing", type=float, default=5.0)
    parser.add_argument("--margin", type=float, default=25.0)
    parser.add_argument("--fill-gaps-metres", type=float, default=0, help="Explicitly interpolate small NoData gaps (0 disables)")
    parser.add_argument("--basemap", action="store_true", help="Download and drape Hamburg GeoBasisKarten")
    args = parser.parse_args()
    if not math.isfinite(args.spacing) or args.spacing < 1 or not math.isfinite(args.margin) or args.margin < 0:
        parser.error("Use spacing >= 1 m and margin >= 0 m")
    if not math.isfinite(args.fill_gaps_metres) or not 0 <= args.fill_gaps_metres <= 20:
        parser.error("Gap interpolation must be between 0 and 20 m")
    metadata = json.loads(args.georef.read_text())
    if metadata["horizontal_crs"] != "EPSG:25832" or metadata["runtime_axes"] != ["west", "up", "north"]:
        parser.error("Unsupported city coordinate reference")
    west, south, east, north = metadata["bounds_utm"]
    s = args.spacing
    bounds = [math.floor((west - args.margin) / s) * s, math.floor((south - args.margin) / s) * s,
              math.ceil((east + args.margin) / s) * s, math.ceil((north + args.margin) / s) * s]
    heights, filled = sample_grid(args.tiles, bounds, s, args.fill_gaps_metres)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    map_url = None
    if args.basemap:
        width = 2048
        height = max(1, round(width * (bounds[3] - bounds[1]) / (bounds[2] - bounds[0])))
        params = dict(SERVICE="WMS", VERSION="1.3.0", REQUEST="GetMap", CRS="EPSG:25832",
                      BBOX=",".join(map(str, bounds)), WIDTH=width, HEIGHT=height,
                      LAYERS="geobasiskarten_farbig", STYLES="", FORMAT="image/png", TRANSPARENT="FALSE")
        map_url = "https://geodienste.hamburg.de/HH_WMS_Geobasiskarten?" + urlencode(params)
        with urlopen(map_url, timeout=90) as response:
            image = response.read(64 * 1024 * 1024 + 1)
        if not image.startswith(b"\x89PNG\r\n\x1a\n") or len(image) > 64 * 1024 * 1024:
            raise ValueError("WMS did not return a PNG within the size limit")
        (args.output_dir / "terrain_map.png").write_bytes(image)
    write_mesh(args.output_dir / "terrain.obj", metadata, bounds, heights, s, args.basemap)
    metadata.update(bounds_utm=bounds, grid_spacing_m=s, terrain_vertical_crs="DE_DHHN2016_NH",
                    terrain_source="Hamburg DGM1 2022", source_tiles=[p.name for p in args.tiles],
                    attribution="Freie und Hansestadt Hamburg, Landesbetrieb Geoinformation und Vermessung (LGV), dl-de/by-2-0; resampled and meshed",
                    basemap_url=map_url, height_range=[float(heights.min()), float(heights.max())])
    metadata.update(interpolated_vertices=filled, gap_fill_search_m=args.fill_gaps_metres)
    (args.output_dir / "terrain.georef.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print("Wrote terrain:", heights.shape, "height range", metadata["height_range"], "bounds", bounds)


if __name__ == "__main__":
    main()
