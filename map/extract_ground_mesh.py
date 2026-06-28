from pathlib import Path

import numpy as np
import trimesh


def main() -> None:
    base = Path(__file__).resolve().parent
    src = base / "surface.obj"
    dst = base / "surface_ground.obj"

    mesh = trimesh.load(src, force="mesh")
    v = np.asarray(mesh.vertices, dtype=np.float64)
    x = v[:, 0]
    y = v[:, 1]
    z = v[:, 2]

    # Grid-based lower envelope (ground proxy)
    cell = 2.0  # meters
    xmin, xmax = float(x.min()), float(x.max())
    zmin, zmax = float(z.min()), float(z.max())

    ix = np.floor((x - xmin) / cell).astype(int)
    iz = np.floor((z - zmin) / cell).astype(int)

    # Keep 10th percentile per cell to suppress roof outliers
    buckets: dict[tuple[int, int], list[float]] = {}
    for i in range(len(v)):
        key = (int(ix[i]), int(iz[i]))
        buckets.setdefault(key, []).append(float(y[i]))

    ground_y: dict[tuple[int, int], float] = {}
    for key, ys in buckets.items():
        arr = np.array(ys, dtype=np.float64)
        ground_y[key] = float(np.percentile(arr, 10.0))

    # Build regularized grid vertices
    keys = list(ground_y.keys())
    if not keys:
        raise RuntimeError("No cells produced for ground mesh")

    verts = []
    index_of: dict[tuple[int, int], int] = {}
    for k in sorted(keys):
        gx = xmin + (k[0] + 0.5) * cell
        gz = zmin + (k[1] + 0.5) * cell
        gy = ground_y[k]
        index_of[k] = len(verts)
        verts.append([gx, gy, gz])

    # Create triangles for adjacent filled quads
    faces = []
    for (i, j), a in index_of.items():
        b_key = (i + 1, j)
        c_key = (i, j + 1)
        d_key = (i + 1, j + 1)
        if b_key in index_of and c_key in index_of and d_key in index_of:
            b = index_of[b_key]
            c = index_of[c_key]
            d = index_of[d_key]
            faces.append([a, b, c])
            faces.append([b, d, c])

    gmesh = trimesh.Trimesh(vertices=np.asarray(verts), faces=np.asarray(faces), process=False)
    dst.write_text(trimesh.exchange.obj.export_obj(gmesh))

    print(f"Wrote {dst}")
    print("Bounds:")
    print(gmesh.bounds)


if __name__ == "__main__":
    main()
