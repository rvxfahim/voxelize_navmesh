# voxelize_navmesh

<video src="https://github.com/rvxfahim/voxelize_navmesh/raw/refs/heads/bugfix/cmd_vel_0/3DNav.mp4" controls width="720"></video>

ROS2 workspace with Recast/Detour as a pinned submodule. The primary feature is
tile-cache dynamic obstacle avoidance: detected point-cloud clusters are fitted
as cylinder obstacles and injected into a live Detour tile cache, while a
`dtCrowd` of agents navigates around them in real time.

## Clone

Use recursive clone so the exact Recast commit for this branch is checked out:

```bash
git clone --recursive https://github.com/rvxfahim/voxelize_navmesh.git
cd voxelize_navmesh
```

If you already cloned without recursive:

```bash
git submodule update --init --recursive
```

## Build

From repository root:

```bash
colcon build --packages-select voxnav
```

`voxnav` builds Recast/Detour internally (with PIC) during colcon — no manual
`cmake` pre-step required.

## Bake a `.tcbin` navmesh

A `.tcbin` file packages a tiled navmesh together with compressed heightfield
layers needed for tile-cache obstacle updates. `blender_y.obj` (Y-up Blender
scene, committed) is the reference input:

```bash
python3 bake_tcbin.py blender_y.obj blender.tcbin
```

## Run — obstacle co-simulation

```bash
source install/setup.bash
ros2 launch voxnav simulate_obstacle_cosim.launch.py \
  tcbin_file:=$(pwd)/blender.tcbin \
  obj_file:=$(pwd)/blender_y.obj
```

Key optional arguments:

| Argument | Default | Description |
|---|---|---|
| `max_linear_speed` | 2.0 | Maximum linear velocity (m/s) |
| `max_angular_speed` | 1.5 | Maximum angular velocity (rad/s) |
| `update_rate` | 30.0 | Simulation update rate (Hz) |
| `robot_radius` | 0.8 | Physical radius of each agent (m) |
| `dyn_obstacle_source` | `cloud` | `cloud` (subscribe to `/foreground_cloud`) or `none` |
| `cylinder_padding` | 0.5 | Extra radius added around each obstacle cluster (m) |
| `height_padding` | 0.55 | Extra height added to each cylinder obstacle (m) |
| `obstacle_decay_s` | 0.5 | Seconds to keep an obstacle after it leaves the point cloud |
| `max_tc_obstacles` | 1024 | Maximum simultaneous tile-cache cylinder obstacles |

## Navmesh baker tool

PyQt/Open3D tool for baking `.bin` and `.tcbin` navmeshes from OBJ input:

```bash
source install/setup.bash
navmesh_baker
```

Or via ROS2 launch:

```bash
source install/setup.bash
ros2 launch voxnav navmesh_baker.launch.py
```

## Layout

```text
.
├── src/
│   ├── voxnav/
│   │   └── launch/simulate_obstacle_cosim.launch.py
│   └── recastnavigation/        # submodule (pinned commit)
├── blender.obj / blender_y.obj  # Blender scene, committed (Y-up variant for Recast)
├── blender.mtl / blender_y.mtl
├── blender.tcbin                # pre-baked tile-cache navmesh
├── bake_tcbin.py
└── 3DNav.mp4
```

## Notes

- Dynamic obstacles subscribe to `/foreground_cloud` (`sensor_msgs/PointCloud2`).
- `blender_y.obj` is the Y-up reorientation of the Blender export; Recast/Detour use Y-up internally.
- Other `.obj` files (voxel meshes, exports) are gitignored.
- Recast build warnings during `colcon build` are non-fatal.
