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
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
```

## Run

```sh
./build/UAM-Simulator-Modern
```

If `map/hh_clip.obj` and `map/hh_clip.mtl` are not tracked locally, regenerate them before running:

```sh
./.venv/bin/python map/extract_clip.py --full-tile
```

That export depends on the local CityGML data under `map/LoD3-HH_Area4_2024_10_10/`, which is intentionally git-ignored.


## Controls

- `Esc`: quit
- `Tab`: switch active/followed sensor drone
- `M`: toggle manual mode for the selected drone
- `V`: toggle selected-drone onboard camera
- `P`: toggle bottom-right selected-drone camera viewport
- `O`: toggle selected-drone camera PNG recording
- `F`: toggle follow/free camera
- `L`: toggle LiDAR debug points
- `R`: toggle RADAR debug points
- `W/A/S/D`: move free camera, or move selected drone in manual mode
- `Space` / `C`: move free camera up/down, or move selected drone up/down in manual mode
- `Arrow keys`: rotate camera
- `Q/E`: rotate camera, or yaw selected drone in manual mode

## Sensor frame export

- `L` writes LiDAR YAML frames to `lidar_output`.
- `R` writes RADAR YAML frames to `radar_output` with range, azimuth, elevation, SNR, object ID, and relative radial velocity.
- `O` writes selected-drone camera PNG frames to `camera_output`; PiP preview alone does not record frames.

## Rendering notes

- Drone body and propeller OBJ meshes are uploaded once to Metal and moved with per-part model matrices.
- Drone-to-drone LiDAR uses separate BVHs for the body and spinning propeller meshes, with a coarse prop-inclusive box as broad phase.
- Selected drones can be flown manually in follow/onboard view; switching to free camera leaves the manual drone paused while camera movement controls the camera. Route following resumes when manual mode is disabled.
- The onboard camera is mounted just ahead of the selected drone and skips rendering that drone to avoid self-occlusion.
- A large bottom-right picture-in-picture viewport can show the selected drone camera while preserving the main camera view; toggle it with `P`. LiDAR debug points are overlaid there when `L` is enabled. Camera PNG recording is separate and toggled with `O`.
- Free camera mode inherits the current view pose when leaving follow/onboard mode, so drone selection does not reset the camera.
- Follow camera is intentionally closer to the lead drone for inspecting the model.
- Static grid geometry is kept in a persistent Metal buffer instead of being rebuilt each frame.
- HUD overlay reports selected drone mode, camera mode, LiDAR/RADAR state, position, velocity, yaw, and FPS.

## Direction

Next implementation steps:

1. Add material/color support for city meshes.
2. Add sensor noise and richer RADAR target/RCS modeling.
3. Move camera/input into dedicated classes.
4. Add Metal compute kernels for LiDAR/RADAR ray batches.
