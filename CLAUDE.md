# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MuJoCo sensor plugin implementing ray-casting-based depth sensors (camera, lidar, generic ray caster). Builds as a shared library (`libsensor_raycaster.so`) that registers three sensor plugins with MuJoCo. Designed to be cloned into MuJoCo's `plugin/` directory.

## Build

The plugin is built as part of MuJoCo's build system. Integration steps:
1. Clone into `mujoco/plugin/mujoco_ray_caster`
2. Add `add_subdirectory(plugin/mujoco_ray_caster)` to MuJoCo's top-level `CMakeLists.txt`
3. Build MuJoCo normally — the plugin compiles alongside it

The built library is copied to the project's `lib/` subdirectory via a post-build step.

Standalone demo builds:
- `demo/C++/` — cmake project finding MuJoCo from `/opt/mujoco/lib/cmake` and OpenCV
- `demo/ROS2/colcon/` — ROS2 colcon package

## Architecture

Two-layer design: plugin glue layer (top-level `.cc/.h`) wraps a standalone algorithm layer (`raycaster_src/`).

**Class hierarchy:**
```
RayCaster (raycaster_src/) → RayCasterCamera → RayCasterLidar
RayPlugin (top-level) → RayCasterPlugin → RayCasterCameraPlugin → RayCasterLidarPlugin
```

Each plugin subclass owns and configures the corresponding core algorithm class.

**Data flow:**
1. MJCF XML defines sensor plugins with config attributes
2. Plugin `init` reads config, creates core object, wires `SensorData` lambdas that write into `mjData.sensordata`
3. Each step: `compute()` → `mj_ray` casts all rays → noise pipeline → loss filtering → sensordata output
4. Plugin state in `mjData.plugin_state` stores metadata (ray counts, data offsets)

**Sensor data types** are composable strings (e.g., `"image inv_image pos_w pos_b"`) — each token maps to a lambda calling a `RayCaster` method.

**Noise pipeline:** Per-ray noise (`produce_noise`) then united noise (`produce_united_noise`) for cross-ray effects (angle-dependent zeros, energy-based stereo loss).

**Version gating:** Features like loss angle, surface normals, and ray energy require `mjVERSION_HEADER > 340` (MuJoCo >= 3.5.0), because `mj_ray` gained a `normal` output parameter in that version.

**Threading:** Multi-threaded ray computation via MuJoCo's `mjThreadPool` (configurable via `num_thread` attribute in MJCF).

## Key Configuration (MJCF Attributes)

Common attributes across all three plugins: `h_ray_num`, `v_ray_num`, `data_type`, `noise_type`, `noise_param`, `loss_angle`, `cont_loss_angle`, `num_thread`, `visualize`. Camera plugin adds: `focal_length`, `aperture`, `baseline`. Lidar plugin adds: `horizontal_fov`, `vertical_fov`.

## Testing

No formal test framework. Manual testing via demo programs and MJCF scene files in `model/` (7+ XML files with various sensor configurations).

## Dependencies

- **MuJoCo** — primary; uses both the public API and internal engine headers (`engine/engine_name.h`, `engine/engine_util_errmem.h`, `engine/engine_util_spatial.h`)
- **OpenCV** — only in C++ demo
- **OpenGL/GLFW/GLU** — only in C++ demo viewer
- **ROS2 + CycloneDDS** — only in ROS2 demo
- **ORB-SLAM3, Orbbec SDK** — only in real2sim workflow

## Standalone API

`raycaster_src/` can be used independently with the MuJoCo C++ API (without the plugin layer). Referenced in the external `go2w_sim2sim` project.