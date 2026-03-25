# voxelize_navmesh (ROS2 branch)

ROS2 workspace with bundled Recast/Detour as a pinned submodule.

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

## Build (single command)

From repository root:

```bash
colcon build --packages-select voxnav
```

`voxnav` now builds Recast/Detour internally (with PIC) during colcon, so no
manual `cmake -S src/recastnavigation ...` pre-step is required.

## Run

```bash
source install/setup.bash
ros2 launch voxnav simulate_crowd.launch.py \
  navmesh_file:=/absolute/path/to/solo_navmesh.bin
```

Example with repo sample:

```bash
ros2 launch voxnav simulate_crowd.launch.py \
  navmesh_file:=$(pwd)/solo_navmesh.bin
```

## Layout

```text
.
├── src/
│   ├── voxnav/
│   └── recastnavigation/   # submodule (pinned commit)
├── solo_navmesh.bin
└── voxel_mesh*.obj
```

## Notes

- `simulate_crowd.launch.py` executes installed `simulate_crowd.py`.
- `voxnav.navmesh` resolves `navmesh_bridge.so` from ROS2 install prefix paths.
- Recast warnings may appear during build; they are non-fatal unless build exits with an error.
