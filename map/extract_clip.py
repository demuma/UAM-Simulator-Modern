import re
import math
import argparse
from pathlib import Path

def main():
    UV_SWAP = False
    UV_FLIP_V = True
    TEXTURE_SUBDIR = "images_upscaled_x4"

    base_dir = Path(__file__).resolve().parent  # map/
    default_tile = base_dir / "LoD3-HH_Area4_2024_10_10" / "6734"
    default_gml = default_tile / "6734.gml"
    default_mtl = base_dir / "hh_clip.mtl"
    default_obj = base_dir / "hh_clip.obj"

    parser = argparse.ArgumentParser(description="Extract CityGML clip to OBJ/MTL")
    parser.add_argument(
        "--gml",
        default=str(default_gml),
        help="Path to the CityGML tile file",
    )
    parser.add_argument(
        "--full-tile",
        action="store_true",
        help="Export the full tile envelope instead of a centered clip window",
    )
    args = parser.parse_args()

    gml_path = Path(args.gml)
    if not gml_path.exists():
        raise SystemExit('GML not found')

    text = gml_path.read_text(errors='ignore')
    tile_dir = gml_path.parent.name
    env = re.search(r'<gml:lowerCorner>([^<]+)</gml:lowerCorner>\s*<gml:upperCorner>([^<]+)</gml:upperCorner>', text, re.S)
    if not env:
        raise SystemExit('Envelope not found')

    lo = list(map(float, env.group(1).split()))
    hi = list(map(float, env.group(2).split()))
    centerE = (lo[0] + hi[0]) * 0.5
    centerN = (lo[1] + hi[1]) * 0.5

    half = 100.0
    minE = centerE - half
    maxE = centerE + half
    minN = centerN - half
    maxN = centerN + half
    if args.full_tile:
        minE, maxE = lo[0], hi[0]
        minN, maxN = lo[1], hi[1]
        half = max(maxE - minE, maxN - minN) * 0.5

    ring_blocks = re.findall(r'<gml:LinearRing[^>]*gml:id="([^"]+)"[^>]*>\s*<gml:posList[^>]*>([^<]+)</gml:posList>', text, re.S)
    ring_pos = {}
    for ring_id, poslist in ring_blocks:
        vals = [float(x) for x in poslist.split() if x.strip()]
        pts = [vals[i:i+3] for i in range(0, len(vals), 3)]
        ring_pos[ring_id] = pts

    # Identify ground surface rings (if present)
    ground_ring_ids = set()
    ground_blocks = re.findall(r'<bldg:GroundSurface[\s\S]*?</bldg:GroundSurface>', text)
    for gb in ground_blocks:
        for m in re.finditer(r'<gml:LinearRing[^>]*gml:id="([^"]+)"', gb):
            ground_ring_ids.add(m.group(1))

    # texture mapping: ring_id -> (image_uri, uvs)
    ring_uv = {}
    tex_blocks = re.findall(r'<app:ParameterizedTexture[\s\S]*?</app:ParameterizedTexture>', text)
    for tb in tex_blocks:
        img = re.search(r'<app:imageURI>([^<]+)</app:imageURI>', tb)
        if not img:
            continue
        image_uri = img.group(1).replace('\\', '/').strip()
        for m in re.finditer(r'<app:textureCoordinates\s+ring="#([^"]+)">([^<]+)</app:textureCoordinates>', tb):
            ring_id = m.group(1)
            uv_vals = [float(x) for x in m.group(2).split() if x.strip()]
            uvs = [(uv_vals[i], uv_vals[i+1]) for i in range(0, len(uv_vals), 2)]
            if UV_SWAP:
                uvs = [(v, u) for (u, v) in uvs]
            if UV_FLIP_V:
                uvs = [(u, 1.0 - v) for (u, v) in uvs]
            ring_uv[ring_id] = (image_uri, uvs)

    def parse_poslist(s):
        vals = [float(x) for x in s.split() if x.strip()]
        return [vals[i:i+3] for i in range(0, len(vals), 3)]

    def inside(pt):
        e, n, _ = pt
        return (minE <= e <= maxE) and (minN <= n <= maxN)

    sumE = 0.0
    sumN = 0.0
    count = 0
    rings = []
    for ring_id, pts in ring_pos.items():
        if len(pts) < 3:
            continue
        for e, n, _ in pts:
            sumE += e
            sumN += n
            count += 1
        if all(inside(p) for p in pts):
            rings.append((ring_id, pts))

    if count > 0 and not args.full_tile:
        centerE = sumE / count
        centerN = sumN / count
        minE = centerE - half
        maxE = centerE + half
        minN = centerN - half
        maxN = centerN + half
        rings = []
        for ring_id, pts in ring_pos.items():
            if len(pts) < 3:
                continue
            if all(inside(p) for p in pts):
                rings.append((ring_id, pts))

    if not rings and not args.full_tile:
        print('No polygons fully inside 200x200m window, expanding to 400x400m')
        half = 200.0
        minE = centerE - half
        maxE = centerE + half
        minN = centerN - half
        maxN = centerN + half
        rings = []
        for ring_id, pts in ring_pos.items():
            if len(pts) < 3:
                continue
            if all(minE <= p[0] <= maxE and minN <= p[1] <= maxN for p in pts):
                rings.append((ring_id, pts))

    if not rings:
        raise SystemExit('No polygons found in window')

    ground_heights = [p[2] for ring_id, poly in rings if ring_id in ground_ring_ids for p in poly]
    if ground_heights:
        minH = min(ground_heights)
    else:
        minH = min(p[2] for _, poly in rings for p in poly)

    # build materials for textures
    images = {}
    for ring_id, _ in rings:
        if ring_id in ring_uv:
            image_uri, _ = ring_uv[ring_id]
            if image_uri not in images:
                images[image_uri] = f"tex_{len(images)}"

    mtl_path = default_mtl
    with mtl_path.open('w') as mtl:
        for image_uri, mat_name in images.items():
            mtl.write(f"newmtl {mat_name}\n")
            mtl.write("Kd 1.000 1.000 1.000\n")
            mtl.write("Ka 0.200 0.200 0.200\n")
            mtl.write("Ks 0.000 0.000 0.000\n")
            mtl.write("d 1.0\n")
            tex_rel = image_uri.replace("images/", f"{TEXTURE_SUBDIR}/")
            # Match upscaler output extension (.png by default)
            if tex_rel.lower().endswith(".jpg") or tex_rel.lower().endswith(".jpeg"):
                tex_rel = str(Path(tex_rel).with_suffix(".png"))
            mtl.write(f"map_Kd LoD3-HH_Area4_2024_10_10/{tile_dir}/{tex_rel}\n\n")

    def area2d(poly2):
        a = 0.0
        for i in range(len(poly2)):
            x0, y0 = poly2[i]
            x1, y1 = poly2[(i + 1) % len(poly2)]
            a += x0 * y1 - x1 * y0
        return 0.5 * a

    def newell_normal(pts):
        nx = ny = nz = 0.0
        for i in range(len(pts)):
            x0, y0, z0 = pts[i]
            x1, y1, z1 = pts[(i + 1) % len(pts)]
            nx += (y0 - y1) * (z0 + z1)
            ny += (z0 - z1) * (x0 + x1)
            nz += (x0 - x1) * (y0 + y1)
        return (nx, ny, nz)

    def project_2d(pts):
        nx, ny, nz = newell_normal(pts)
        ax, ay, az = abs(nx), abs(ny), abs(nz)
        if ax >= ay and ax >= az:
            return [(p[1], p[2]) for p in pts]  # YZ
        if ay >= az:
            return [(p[0], p[2]) for p in pts]  # XZ
        return [(p[0], p[1]) for p in pts]      # XY

    def area2d_uv(uvs):
        a = 0.0
        for i in range(len(uvs)):
            x0, y0 = uvs[i]
            x1, y1 = uvs[(i + 1) % len(uvs)]
            a += x0 * y1 - x1 * y0
        return 0.5 * a

    def point_in_tri(pt, a, b, c):
        px, py = pt
        ax, ay = a
        bx, by = b
        cx, cy = c
        v0x, v0y = cx - ax, cy - ay
        v1x, v1y = bx - ax, by - ay
        v2x, v2y = px - ax, py - ay
        dot00 = v0x * v0x + v0y * v0y
        dot01 = v0x * v1x + v0y * v1y
        dot02 = v0x * v2x + v0y * v2y
        dot11 = v1x * v1x + v1y * v1y
        dot12 = v1x * v2x + v1y * v2y
        denom = dot00 * dot11 - dot01 * dot01
        if abs(denom) < 1e-12:
            return False
        inv = 1.0 / denom
        u = (dot11 * dot02 - dot01 * dot12) * inv
        v = (dot00 * dot12 - dot01 * dot02) * inv
        return (u >= -1e-8) and (v >= -1e-8) and (u + v <= 1.0 + 1e-8)

    def triangulate(poly, uvs):
        if len(poly) < 3:
            return []
        poly2 = list(uvs)
        if area2d(poly2) < 0.0:
            poly = list(reversed(poly))
            uvs = list(reversed(uvs))
            poly2 = list(reversed(poly2))

        idxs = list(range(len(poly)))
        tris = []
        guard = 0
        while len(idxs) >= 3 and guard < 10000:
            guard += 1
            ear_found = False
            for i in range(len(idxs)):
                i0 = idxs[(i - 1) % len(idxs)]
                i1 = idxs[i]
                i2 = idxs[(i + 1) % len(idxs)]
                a = poly2[i0]
                b = poly2[i1]
                c = poly2[i2]
                cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
                if cross <= 1e-12:
                    continue
                is_ear = True
                for j in idxs:
                    if j in (i0, i1, i2):
                        continue
                    if point_in_tri(poly2[j], a, b, c):
                        is_ear = False
                        break
                if not is_ear:
                    continue
                tris.append((i0, i1, i2))
                idxs.pop(i)
                ear_found = True
                break
            if not ear_found:
                break
        return [(poly[i0], poly[i1], poly[i2], uvs[i0], uvs[i1], uvs[i2]) for (i0, i1, i2) in tris]

    out_path = default_obj
    with out_path.open('w') as f:
        f.write('# Hamburg CityGML clip\n')
        f.write(f'# centerE {centerE} centerN {centerN} half {half}\n')
        f.write('mtllib hh_clip.mtl\n')
        v_idx = 1
        vt_idx = 1
        vn_idx = 1
        winding_mismatches = 0
        winding_checked = 0
        for ring_id, poly in rings:
            if len(poly) >= 2 and all(abs(poly[0][i]-poly[-1][i]) < 1e-6 for i in range(3)):
                poly = poly[:-1]
            if len(poly) < 3:
                continue

            if ring_id in ring_uv:
                image_uri, uvs = ring_uv[ring_id]
                if len(uvs) >= 2:
                    if abs(uvs[0][0] - uvs[-1][0]) < 1e-6 and abs(uvs[0][1] - uvs[-1][1]) < 1e-6:
                        uvs = uvs[:-1]
            else:
                image_uri, uvs = None, None

            if uvs is None:
                continue

            if len(uvs) == len(poly) + 1:
                uvs = uvs[:-1]
            if len(poly) == len(uvs) + 1:
                poly = poly[:-1]
            if len(uvs) != len(poly):
                continue

            # Diagnostic: original winding consistency between geometry and UVs
            poly2_geom = project_2d(poly)
            if poly2_geom:
                winding_checked += 1
                if area2d(poly2_geom) * area2d_uv(uvs) < 0.0:
                    winding_mismatches += 1

            # Keep original UV order/mapping from source (no heuristic reordering).

            mat_name = images.get(image_uri)
            if mat_name:
                f.write(f"usemtl {mat_name}\n")

            triangles = triangulate(poly, uvs)
            if not triangles:
                continue
            for p0, p1, p2, uv0, uv1, uv2 in triangles:
                tri = [p0, p1, p2]
                tri_uv = [uv0, uv1, uv2]

                def to_local(p):
                    e, n, h = p
                    x = e - centerE
                    y = h - minH
                    z = n - centerN
                    return (x, y, z)

                v0 = to_local(tri[0])
                v1 = to_local(tri[1])
                v2 = to_local(tri[2])

                ux, uy, uz = (v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2])
                vx, vy, vz = (v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2])
                nx, ny, nz = (uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx)
                l = math.sqrt(nx*nx+ny*ny+nz*nz) or 1.0
                nx, ny, nz = (nx/l, ny/l, nz/l)

                for v in (v0, v1, v2):
                    f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
                for uv in tri_uv:
                    u, v = uv
                    f.write(f"vt {u:.6f} {v:.6f}\n")
                for _ in range(3):
                    f.write(f"vn {nx:.6f} {ny:.6f} {nz:.6f}\n")

                f.write(f"f {v_idx}/{vt_idx}/{vn_idx} {v_idx+1}/{vt_idx+1}/{vn_idx+1} {v_idx+2}/{vt_idx+2}/{vn_idx+2}\n")
                v_idx += 3
                vt_idx += 3
                vn_idx += 3

    print('Wrote', out_path, 'triangles', (v_idx-1)//3,
          'winding_checked', winding_checked,
          'winding_mismatches', winding_mismatches,
          'uv_swap', UV_SWAP,
          'uv_flip_v', UV_FLIP_V)

if __name__ == '__main__':
    main()
