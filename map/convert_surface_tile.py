import itertools
import json
import math
import re
from pathlib import Path

import numpy as np
import trimesh
from pyproj import Transformer


def _find_region_for_uri(tileset_path: Path, target_uri: str):
    target_uri = target_uri.replace("\\", "/")
    visited = set()

    def walk_json(ts_path: Path):
        resolved = ts_path.resolve()
        if resolved in visited:
            return None
        visited.add(resolved)

        data = json.loads(ts_path.read_text())
        stack = [data["root"]]
        while stack:
            node = stack.pop()
            content = node.get("content", {})
            uri = content.get("uri", "")
            region = node.get("boundingVolume", {}).get("region")
            if uri.replace("\\", "/") == target_uri and region is not None:
                return region
            if uri.lower().endswith(".json"):
                nested = (ts_path.parent / uri).resolve()
                if nested.exists():
                    found = walk_json(nested)
                    if found is not None:
                        return found
            stack.extend(node.get("children", []))
        return None

    return walk_json(tileset_path)


def _enu_to_ecef_matrix(lon_rad: float, lat_rad: float) -> np.ndarray:
    sl, cl = math.sin(lon_rad), math.cos(lon_rad)
    sp, cp = math.sin(lat_rad), math.cos(lat_rad)
    east = np.array([-sl, cl, 0.0], dtype=np.float64)
    north = np.array([-sp * cl, -sp * sl, cp], dtype=np.float64)
    up = np.array([cp * cl, cp * sl, sp], dtype=np.float64)
    return np.column_stack([east, north, up])


def _axis_candidates() -> list[tuple[tuple[int, int, int], tuple[int, int, int]]]:
    perms = list(itertools.permutations((0, 1, 2), 3))
    signs = list(itertools.product((-1, 1), repeat=3))
    return [(p, s) for p in perms for s in signs]


def main() -> None:
    base = Path(__file__).resolve().parent

    # Inputs for currently selected terrain content
    b3dm_uri = "9a/18-138367-84716.b3dm"
    tileset_path = base / "tile006-006-3dtiles" / "tileset.json"
    feature_table_path = base / "_analyze" / "18-138367-84716.b3dm.featureTable.json"
    glb_path = base / "surface_source.glb"
    gml_path = base / "LoD3-HH_Area4_2024_10_10" / "6734" / "6734.gml"
    out_obj = base / "surface.obj"

    feature_table = json.loads(feature_table_path.read_text())
    rtc_center = np.array(feature_table["RTC_CENTER"], dtype=np.float64)

    # Load as scene to include node transforms, then concatenate to one mesh
    scene = trimesh.load(glb_path, force="scene")
    mesh = scene.dump(concatenate=True)
    vertices = np.asarray(mesh.vertices, dtype=np.float64)

    # Read expected tile extents from tileset region
    region = _find_region_for_uri(tileset_path, b3dm_uri)
    if region is None:
        raise RuntimeError(f"Could not find region for {b3dm_uri}")
    west, south, east, north, min_h_reg, max_h_reg = region
    mean_lat = 0.5 * (south + north)
    expected_dx = abs(east - west) * 6378137.0 * math.cos(mean_lat)
    expected_dz = abs(north - south) * 6378137.0
    expected_dy = abs(max_h_reg - min_h_reg)

    # ECEF center -> geodetic for ENU basis
    tr_ecef_to_geo = Transformer.from_crs("EPSG:4978", "EPSG:4979", always_xy=True)
    lon_deg, lat_deg, _ = tr_ecef_to_geo.transform(rtc_center[0], rtc_center[1], rtc_center[2])
    lon_rad, lat_rad = math.radians(lon_deg), math.radians(lat_deg)
    enu_to_ecef = _enu_to_ecef_matrix(lon_rad, lat_rad)

    tr_ecef_to_utm = Transformer.from_crs("EPSG:4978", "EPSG:25832", always_xy=True)

    # Read city anchor (same as extract_clip.py)
    gml_text = gml_path.read_text(errors="ignore")
    env = re.search(
        r"<gml:lowerCorner>([^<]+)</gml:lowerCorner>\s*<gml:upperCorner>([^<]+)</gml:upperCorner>",
        gml_text,
        re.S,
    )
    if not env:
        raise RuntimeError("Envelope not found in CityGML")
    lo = np.array(list(map(float, env.group(1).split())))
    hi = np.array(list(map(float, env.group(2).split())))
    center_e = (lo[0] + hi[0]) * 0.5
    center_n = (lo[1] + hi[1]) * 0.5
    min_h_city = lo[2]

    best_score = float("inf")
    best_local = None
    best_cfg = None

    for perm, sign in _axis_candidates():
        # Candidate mapping from glTF local axes -> ENU
        enu = np.column_stack([
            sign[0] * vertices[:, perm[0]],
            sign[1] * vertices[:, perm[1]],
            sign[2] * vertices[:, perm[2]],
        ])

        # ENU -> ECEF around RTC center
        ecef = rtc_center[None, :] + enu @ enu_to_ecef.T
        easting, northing, height = tr_ecef_to_utm.transform(ecef[:, 0], ecef[:, 1], ecef[:, 2])

        local = np.column_stack([easting - center_e, height - min_h_city, northing - center_n])
        mn = local.min(axis=0)
        mx = local.max(axis=0)
        rx, ry, rz = (mx - mn).tolist()

        # Match expected footprint/height from tileset region
        score = (
            abs(rx - expected_dx)
            + abs(rz - expected_dz)
            + 8.0 * abs(ry - expected_dy)
        )
        if score < best_score:
            best_score = score
            best_local = local
            best_cfg = (perm, sign, (rx, ry, rz))

    if best_local is None:
        raise RuntimeError("No conversion candidate found")

    # Many tiles contain non-ground structures. For a surface-height proxy,
    # clamp high outliers to the expected vertical span from tile metadata.
    y = best_local[:, 1]
    y_base = np.percentile(y, 2.0)
    y_top = y_base + expected_dy
    best_local[:, 1] = np.clip(y, y_base, y_top)

    mesh.vertices = best_local
    out_obj.write_text(trimesh.exchange.obj.export_obj(mesh))

    perm, sign, ranges = best_cfg
    print(f"Wrote {out_obj}")
    print(f"Selected axis mapping perm={perm}, sign={sign}")
    print(f"Expected ranges [dx,dy,dz]=[{expected_dx:.3f}, {expected_dy:.3f}, {expected_dz:.3f}]")
    print(f"Result ranges   [rx,ry,rz]=[{ranges[0]:.3f}, {ranges[1]:.3f}, {ranges[2]:.3f}]")
    print(f"Height clip     [y_base,y_top]=[{y_base:.3f}, {y_top:.3f}]")
    print("Bounds (local):")
    print(mesh.bounds)


if __name__ == "__main__":
    main()
