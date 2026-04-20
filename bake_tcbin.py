#!/usr/bin/env python3
"""
Bake a tiled navmesh + dtTileCache from an OBJ file and save as .tcbin.

The .tcbin format is what simulate_obstacle_cosim.py loads at startup.
It encodes the tiled navmesh geometry and compressed heightfield layers needed
for dynamic obstacle injection (dtTileCache cylinder obstacles).

Usage:
    python bake_tcbin.py <yup_obj> <output_tcbin> [options]

    <yup_obj>       OBJ file in Y-up coordinates (Blender exports Z-up by default;
                    the _y.obj files in this repo have already been converted).
    <output_tcbin>  Output path for the .tcbin file.

Examples:
    python bake_tcbin.py blender_y.obj blender.tcbin
    python bake_tcbin.py blender_y.obj blender.tcbin --tile-size 32 --cell-size 0.1
    python bake_tcbin.py blender_y.obj blender.tcbin --agent-radius 0.4 --max-obstacles 128

Z-up input (e.g. raw Blender export):
    If your OBJ is Z-up (not _y.obj), pass --zup and the bake will convert internally:
    python bake_tcbin.py blender.obj blender.tcbin --zup
"""

import argparse
import os
import sys

# Allow running from repo root without installing the package.
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "src", "voxnav"))
sys.path.insert(0, os.path.join(_HERE, "src", "voxnav", "voxnav"))


def main():
    parser = argparse.ArgumentParser(
        description="Bake a tiled navmesh (.tcbin) from an OBJ mesh.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("obj", help="Path to OBJ file (Y-up unless --zup is set)")
    parser.add_argument("out", help="Output .tcbin path")

    # Tile cache params
    parser.add_argument("--tile-size", type=int, default=32,
                        help="Tile size in voxels (32 → 3.2 m tiles at cs=0.1)")
    parser.add_argument("--max-obstacles", type=int, default=128,
                        help="Max simultaneous cylinder obstacles")

    # Agent / navmesh params
    parser.add_argument("--cell-size", type=float, default=0.1,
                        help="Voxel cell size (metres)")
    parser.add_argument("--cell-height", type=float, default=0.1,
                        help="Voxel cell height (metres)")
    parser.add_argument("--agent-height", type=float, default=1.0,
                        help="Agent height (metres)")
    parser.add_argument("--agent-radius", type=float, default=0.3,
                        help="Agent radius (metres)")
    parser.add_argument("--agent-max-climb", type=float, default=0.15,
                        help="Max step height (metres)")
    parser.add_argument("--agent-max-slope", type=float, default=45.0,
                        help="Max walkable slope (degrees)")

    # Misc
    parser.add_argument("--zup", action="store_true",
                        help="Input OBJ is Z-up (will be swapped to Y-up internally)")

    args = parser.parse_args()

    if not os.path.isfile(args.obj):
        sys.exit(f"ERROR: OBJ file not found: {args.obj}")

    obj_path = os.path.abspath(args.obj)
    out_path = os.path.abspath(args.out)

    print(f"OBJ  : {obj_path}")
    print(f"Out  : {out_path}")
    print(f"Tiles: size={args.tile_size} voxels, max_obstacles={args.max_obstacles}")
    print(f"Agent: h={args.agent_height} r={args.agent_radius} "
          f"climb={args.agent_max_climb} slope={args.agent_max_slope}")
    print(f"Cell : cs={args.cell_size} ch={args.cell_height}")
    print(f"Input coords: {'Z-up (converting)' if args.zup else 'Y-up'}")
    print()

    from voxnav.navmesh import TileCache
    from voxnav._ctypes_bindings import nmBuildSettings

    s = nmBuildSettings(
        cellSize=args.cell_size,
        cellHeight=args.cell_height,
        agentHeight=args.agent_height,
        agentRadius=args.agent_radius,
        agentMaxClimb=args.agent_max_climb,
        agentMaxSlope=args.agent_max_slope,
        regionMinSize=8,
        regionMergeSize=20,
        edgeMaxLen=12,
        edgeMaxError=1.3,
        vertsPerPoly=6,
        detailSampleDist=6,
        detailSampleMaxError=1,
        partitionType=0,
        filterLowHangingObstacles=1,
        filterLedgeSpans=1,
        filterWalkableLowHeightSpans=1,
    )

    print("Baking tiled navmesh (this may take a few seconds)...")
    tc = TileCache(
        obj_path=obj_path,
        settings=s,
        tile_size=args.tile_size,
        max_obstacles=args.max_obstacles,
        input_is_z_up=args.zup,
    )

    tile_count = tc.navmesh.tile_count
    print(f"Baked: {tile_count} tiles")

    print(f"Saving to {out_path} ...")
    tc.save(out_path)
    tc.close()

    size_kb = os.path.getsize(out_path) / 1024
    print(f"Done. File size: {size_kb:.1f} KB")
    print()
    print("Launch the obstacle cosim node with:")
    print(f"  ros2 run voxnav simulate_obstacle_cosim {out_path}")


if __name__ == "__main__":
    main()
