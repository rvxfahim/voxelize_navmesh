"""
Voxelize Point Cloud → OBJ Export

Loads a PCD file, voxelizes it, generates a triangle mesh via exposed-face
culling, and exports an OBJ file.
"""

import argparse
from voxel_navmesh import (load_pcd, voxelize, export_obj,
                            reorient_to_yup, MESH_METHODS)


def main():
    parser = argparse.ArgumentParser(description="Voxelize a PCD and export OBJ")
    parser.add_argument("pcd", nargs="?", default="fhparking.pcd", help="Input PCD file")
    parser.add_argument("obj", nargs="?", default="voxel_mesh.obj", help="Output OBJ file")
    parser.add_argument("--voxel-size", type=float, default=None,
                        help="Manual voxel size (default: auto = extent.max() / 200)")
    parser.add_argument("--method", choices=MESH_METHODS.keys(), default="greedy",
                        help="Meshing method: naive, greedy, marching_cubes (default: greedy)")
    parser.add_argument("--up-axis", choices=["x", "y", "z"], default=None,
                        help="Source up axis of the PCD (x/y/z). "
                             "Points are reoriented to Y-up before voxelization. "
                             "Implies no axis swap at export. (default: no reorientation)")
    parser.add_argument("--no-swap-yz", action="store_true",
                        help="Disable Y↔Z axis swap in exported OBJ")
    args = parser.parse_args()

    _, points = load_pcd(args.pcd)

    # Pre-process: reorient so Y is up before voxelization
    if args.up_axis:
        points = reorient_to_yup(points, args.up_axis)
        swap_yz = False  # already Y-up, no swap needed at export
    else:
        swap_yz = not args.no_swap_yz

    occupied, bb_min, voxel_size = voxelize(points, voxel_size=args.voxel_size)
    mesh_fn = MESH_METHODS[args.method]
    verts, faces = mesh_fn(occupied, bb_min, voxel_size)
    export_obj(verts, faces, args.obj, swap_yz=swap_yz)
    print("Done.")


if __name__ == "__main__":
    main()
