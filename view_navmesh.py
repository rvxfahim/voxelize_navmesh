"""
view_navmesh.py — Overlay a Recast navmesh (.bin) on a point cloud in Open3D.

Usage
-----
    python3 view_navmesh.py <scene.pcd> <navmesh.bin>

    # Example (using the test navmesh baked from voxel_mesh_yup.obj):
    python3 view_navmesh.py clinic.pcd test_navmesh.bin

Keyboard shortcuts in the viewer
---------------------------------
    H          – print help
    P          – toggle point cloud on/off
    W          – toggle navmesh wireframe on/off
    Q / Esc    – quit

Coordinate frame
----------------
    The point cloud and navmesh are both shown in Z-up space.
    navmesh.py handles the Y↔Z swap from Recast's Y-up convention
    transparently, so the two datasets should align directly.
"""

import sys
import argparse
import numpy as np
import open3d as o3d

from navmesh import NavMesh


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def make_navmesh_mesh(nm: NavMesh) -> o3d.geometry.TriangleMesh:
    """Build an Open3D TriangleMesh from the navmesh detail geometry (Z-up)."""
    verts, tris = nm.get_geometry(swap_yz=True)
    if len(verts) == 0:
        raise RuntimeError("Navmesh has no geometry")

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(verts.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(tris.astype(np.int32))
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.0, 0.85, 0.35])   # green
    return mesh


def make_navmesh_wireframe(mesh: o3d.geometry.TriangleMesh,
                           color=(0.0, 0.6, 0.2)) -> o3d.geometry.LineSet:
    """Extract wireframe edges from the navmesh triangle mesh."""
    ls = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    ls.paint_uniform_color(color)
    return ls


def make_tile_bounds_lines(nm: NavMesh,
                           color=(1.0, 0.5, 0.0)) -> o3d.geometry.LineSet:
    """Draw each tile's AABB as an orange box."""
    points, lines, colors = [], [], []
    pt_off = 0
    for ti in range(nm.tile_count):
        bmin, bmax = nm.tile_bounds(ti)
        # 8 corners of the AABB
        corners = np.array([
            [bmin[0], bmin[1], bmin[2]],
            [bmax[0], bmin[1], bmin[2]],
            [bmax[0], bmax[1], bmin[2]],
            [bmin[0], bmax[1], bmin[2]],
            [bmin[0], bmin[1], bmax[2]],
            [bmax[0], bmin[1], bmax[2]],
            [bmax[0], bmax[1], bmax[2]],
            [bmin[0], bmax[1], bmax[2]],
        ])
        edges = [(0,1),(1,2),(2,3),(3,0),
                 (4,5),(5,6),(6,7),(7,4),
                 (0,4),(1,5),(2,6),(3,7)]
        points.extend(corners.tolist())
        for a, b in edges:
            lines.append([pt_off + a, pt_off + b])
            colors.append(list(color))
        pt_off += 8

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    ls.lines  = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(colors)
    return ls


# ---------------------------------------------------------------------------
# Coordinate frame axes
# ---------------------------------------------------------------------------
def make_axes(size: float = 1.0) -> o3d.geometry.LineSet:
    """Small RGB XYZ axis indicator at the origin."""
    pts = [[0,0,0],[size,0,0],[0,0,0],[0,size,0],[0,0,0],[0,0,size]]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines  = o3d.utility.Vector2iVector([[0,1],[2,3],[4,5]])
    ls.colors = o3d.utility.Vector3dVector([[1,0,0],[0,1,0],[0,0,1]])
    return ls


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Visualise a Recast navmesh (.bin) overlaid on a point cloud")
    parser.add_argument("pcd",  help="Input PCD file (Z-up)")
    parser.add_argument("bin",  help="Navmesh .bin saved by RecastDemo or recast_cli")
    parser.add_argument("--no-pcd",       action="store_true",
                        help="Hide point cloud on startup")
    parser.add_argument("--no-wireframe", action="store_true",
                        help="Show solid navmesh instead of wireframe")
    parser.add_argument("--point-size", type=float, default=1.5,
                        help="Point cloud render size (default 1.5)")
    args = parser.parse_args()

    # --- Load point cloud ---
    print(f"Loading PCD: {args.pcd}")
    pcd = o3d.io.read_point_cloud(args.pcd)
    if len(pcd.points) == 0:
        print(f"ERROR: no points loaded from {args.pcd}", file=sys.stderr)
        sys.exit(1)
    pts = np.asarray(pcd.points)
    print(f"  {len(pts)} points  "
          f"bbox Z-up: [{pts.min(0).round(2)}] – [{pts.max(0).round(2)}]")

    # --- Load navmesh ---
    print(f"Loading navmesh: {args.bin}")
    nm = NavMesh(args.bin)
    print(f"  {nm.tile_count} tile(s)")
    verts, tris = nm.get_geometry(swap_yz=True)
    print(f"  {len(verts)} detail verts,  {len(tris)} detail tris")
    for ti in range(nm.tile_count):
        bmin, bmax = nm.tile_bounds(ti)
        print(f"  tile {ti} bounds (Z-up): {bmin.round(2)} – {bmax.round(2)}")

    # --- Build Open3D objects ---
    navmesh_solid = make_navmesh_mesh(nm)
    navmesh_wire  = make_navmesh_wireframe(navmesh_solid)
    tile_boxes    = make_tile_bounds_lines(nm)
    axes          = make_axes(size=max(float(np.ptp(pts)), 1.0) * 0.05)

    # --- Visualise ---
    geoms = []
    if not args.no_pcd:
        geoms.append(pcd)

    if args.no_wireframe:
        geoms.append(navmesh_solid)
    else:
        geoms.append(navmesh_wire)

    geoms += [tile_boxes, axes]

    print("\nOpening viewer …")
    print("  Navmesh polygons are drawn in green (wireframe by default).")
    print("  Orange boxes = tile AABBs.   RGB axes = XYZ at origin.")
    print("  If the navmesh doesn't overlap the PCD, check which OBJ was")
    print("  used for baking and whether it was exported with --up-axis z.")

    o3d.visualization.draw_geometries(
        geoms,
        window_name=f"NavMesh | {args.bin}",
        width=1280, height=800,
        point_show_normal=False,
    )

    nm.close()


if __name__ == "__main__":
    main()
