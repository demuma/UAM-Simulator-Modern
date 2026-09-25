# UAM Simulator Modern

This is the modern backend experiment for the UAM simulator. The goal is to keep simulation logic in C++ while replacing the legacy SFML/GLEW/OpenGL rendering path with a native Metal backend on macOS.

## Current milestone

- Native Cocoa + MetalKit window.
- C++ simulation core loaded from `config.yaml` and `sensors.yaml`.
- Multiple drones move along configured routes.
- Perspective 3D Metal debug view with depth buffering, a ground grid, textured city OBJ meshes, terrain OBJ meshes, directional sun shading with a shadow map, animated OBJ drone models with visible propeller sweep markers, CPU BVH LiDAR/RADAR debug points, mesh-based drone-to-drone sensor returns, and a HUD overlay.

This is intentionally not a line-by-line port of the old `src/main.cpp`. The old simulator remains the behavior reference while this project grows a cleaner architecture.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

## Run

```sh
./build/UAM-Simulator-Modern
```

Use Release for interactive sensor runs. Unoptimized Debug builds make the CPU BVH scans substantially slower. In VS Code, the default build task is Build Release; select the Run launch configuration for an optimized build. Debug remains available for stepping through code.

If `map/hh_clip.obj` and `map/hh_clip.mtl` are not tracked locally, regenerate them before running:

```sh
./.venv/bin/python map/extract_clip.py --full-tile
```

That export depends on the local CityGML data under `map/LoD3-HH_Area4_2024_10_10/`, which is intentionally git-ignored.


## Controls

- `Esc`: quit
- `Tab`: switch active/followed sensor drone
- `M`: toggle manual mode for the selected drone (enabling it enters follow view)
- `V`: toggle selected-drone onboard camera
- `P`: toggle bottom-right selected-drone camera viewport
- `O`: toggle selected-drone camera PNG recording
- `F`: toggle follow/free camera
- `L`: toggle LiDAR debug points
- `R`: toggle RADAR debug points
- `W/A/S/D`: move free camera, or move selected drone in manual mode
- `Space` / `C`: move free camera up/down, or move selected drone up/down in manual mode
- `Arrow keys`: rotate camera
- Left or right mouse drag: rotate the free/follow camera; onboard view stays fixed to the drone
- `Q/E`: rotate camera, or yaw selected drone in manual mode

## Sensor frame export

- `L` writes LiDAR YAML frames to `lidar_output`.
- `R` writes RADAR YAML frames to `radar_output` with range, azimuth, elevation, SNR, object ID, and relative radial velocity.
- `O` writes selected-drone camera PNG frames to `camera_output`; PiP preview alone does not record frames.

LiDAR scans and YAML export run on a background worker using a snapshot of drone poses. Only one scan runs at a time; when processing cannot keep up, the scan rate falls instead of queuing work and freezing controls. Displayed points describe the captured pose, not the current pose of a moving target. Disabling LiDAR lets an already-started export finish.

## Rendering notes

- Drone body and propeller OBJ meshes are uploaded once to Metal and moved with per-part model matrices.
- Drone-to-drone LiDAR uses separate BVHs for the body and spinning propeller meshes, with a coarse prop-inclusive box as broad phase.
- Selected drones can be flown manually in follow/onboard view; switching to free camera leaves the manual drone paused while camera movement controls the camera. Route following resumes when manual mode is disabled.
- The onboard camera is mounted just ahead of the selected drone and skips rendering that drone to avoid self-occlusion.
- A large bottom-right picture-in-picture viewport can show the selected drone camera while preserving the main camera view; toggle it with `P`. LiDAR debug points are overlaid there when `L` is enabled. Camera PNG recording is separate and toggled with `O`.
- Free camera mode inherits the current view pose when leaving follow/onboard mode, so drone selection does not reset the camera.
- Follow camera is intentionally closer to the lead drone for inspecting the model.
- Static grid geometry is kept in a persistent Metal buffer instead of being rebuilt each frame.
- LiDAR point buffers are uploaded only when a scan completes. Points are 2 pixels in the main view and 1 pixel in the camera preview/output.
- Draw uniforms are copied into Metal commands so overlapping frames cannot overwrite each other's transforms or point sizes.
- Both Objective-C++ files use ARC so temporary images and replaced Metal resources are released.
- RADAR markers use 3 pixels in the main view and 1.5 pixels in the camera preview/output.

## Hamburg terrain and coordinates

The renderer loads `map/terrain.obj` when its coordinate sidecar matches `map/hh_clip.georef.json`. This measured terrain also enters the LiDAR/RADAR BVH. Missing, mismatched, or undersized terrain falls back to the synthetic fill mesh with a console warning. The old 3D-Tiles axis-guessing converter is not used by this path.

The source horizontal CRS is **ETRS89 / UTM zone 32N (EPSG:25832)**, in metres. The building exporter writes east/up/north OBJ coordinates; the renderer mirrors X for the existing west/up/north simulation frame:

```text
E = origin_easting - local_x
N = origin_northing + local_z
H = origin_height + local_y
```

For tile 6734 the origin is `(567488.2015, 5934619.782, 1.917)`. The HUD shows UTM E/N and source height; sensor YAML includes the frame, origin, and sensor pose. Keep geodetic calculations in double precision and subtract the origin before sending coordinates to the renderer. Existing drone positions/routes remain local metres.

The DGM source specifies **DHHN2016 / NHN** heights. The supplied CityGML declares only EPSG:25832, without an explicit vertical CRS; matching its heights to NHN is an assumption, recorded in the sidecars. No ellipsoid-to-NHN conversion or height clamping is performed. Verify the LoD3 vertical datum before using this for survey-accuracy comparisons.

The installed terrain uses Hamburg DGM1 2022, resampled to a **5 m mesh**, with a 25 m margin. This is not the DGM5H hybrid product and does not incorporate its breaklines. Six NoData vertices were interpolated with an explicit 10 m search radius, recorded in `terrain.georef.json`. The mesh has 73,960 triangles. Use `--spacing 1` or `--spacing 2` for denser geometry at increased rendering/BVH cost.

GeoBasisKarten is draped as a north-up map texture over those heights. It supplies cartographic appearance, not geometry or physical radar reflectivity. Existing upscaled building textures remain in use; upscaling cannot add measured terrain detail. For a photographic ground appearance, orthophotos would be the alternative to the city map.

To regenerate the current terrain:

```sh
./.venv/bin/python -m pip install -r map/requirements-terrain.txt
./.venv/bin/python map/extract_clip.py --full-tile --metadata-only
./.venv/bin/python map/fetch_dgm.py \
  --pattern '*dgm1_32_566_5934_*.tif' --pattern '*dgm1_32_566_5935_*.tif' \
  --pattern '*dgm1_32_567_5934_*.tif' --pattern '*dgm1_32_567_5935_*.tif' \
  --pattern '*dgm1_32_568_5934_*.tif' --pattern '*dgm1_32_568_5935_*.tif'
./.venv/bin/python map/build_terrain.py map/dgm/*.tif --basemap --fill-gaps-metres 10
./.venv/bin/python tests/test_terrain.py
```

Skip `fetch_dgm.py` when the tiles already exist. It uses HTTP ranges and ZIP CRC validation to retrieve only selected tiles. No network access is needed at simulator runtime. Generated assets are ignored by Git; regenerate them on a new checkout. `extract_clip.py` also writes the coordinate sidecar during normal OBJ export. After changing the city clip, regenerate the terrain too.

Sources and attribution: Freie und Hansestadt Hamburg, Landesbetrieb Geoinformation und Vermessung (LGV), **Datenlizenz Deutschland Namensnennung 2.0**. Terrain has been resampled, small gaps interpolated, converted to a mesh and textured.

- [Current DGM metadata](https://metaver.de/trefferanzeige?docuuid=aeca28b5-503e-4e69-b796-eb0fbebbad0a)
- [DGM1 2022 archive](https://www.daten-hamburg.de/opendata/fernerkundung_hoehenmodelle/dgm/dgm1_hh_2022-04-30.zip)
- [GeoBasisKarten metadata](https://metaver.de/trefferanzeige?docuuid=B6A59A2B-2D40-4676-9094-0EB73039ED34)
- [GeoBasisKarten WMS capabilities](https://geodienste.hamburg.de/HH_WMS_Geobasiskarten?SERVICE=WMS&REQUEST=GetCapabilities)
- HUD overlay reports selected drone mode, camera mode, LiDAR/RADAR state, position, velocity, yaw, and FPS.

## Direction

Next implementation steps:

1. Add material/color support for city meshes.
2. Add sensor noise and richer RADAR target/RCS modeling.
3. Move camera/input into dedicated classes.
4. Add Metal compute kernels for LiDAR/RADAR ray batches.
