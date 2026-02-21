"""
query_path.py — Interactive or batch Recast/Detour path query.

Interactive mode  (omit start/end coords)
------------------------------------------
    python3 query_path.py <navmesh.bin> --pcd <scene.pcd>
    python3 query_path.py <navmesh.bin>      # navmesh surface used for picking

    Picking controls
    ----------------
        Shift + left-click  – select a point  (first = START, second = END)
        Q / close window    – confirm selection and compute path

Batch mode  (explicit coords)
------------------------------
    python3 query_path.py <navmesh.bin> <sx> <sy> <sz> <ex> <ey> <ez> [opts]

All coordinates are in **Z-up** space (same frame as the point cloud / Open3D).

Options
-------
    --pcd <file>       Overlay PCD (required in interactive mode for full scene)
    --extents x y z    Poly-snap half-extents in Z-up (default: 2 4 2)
    --max-nodes N      A* node budget (default: 2048)
    --no-vis           (batch only) skip the result visualisation
"""

import sys
import argparse
import numpy as np

from navmesh import NavMesh, NavMeshQuery


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _make_navmesh_mesh(nm):
    import open3d as o3d
    verts, tris = nm.get_geometry(swap_yz=True)
    if len(verts) == 0:
        raise RuntimeError("Navmesh has no geometry")
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(verts.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(tris.astype(np.int32))
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.0, 0.85, 0.35])
    return mesh


def _make_wireframe(mesh, color=(0.0, 0.6, 0.2)):
    import open3d as o3d
    ls = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    ls.paint_uniform_color(color)
    return ls


def _make_sphere(center, radius=0.2, color=(1.0, 1.0, 1.0)):
    import open3d as o3d
    s = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    s.translate(np.asarray(center, dtype=np.float64))
    s.paint_uniform_color(color)
    s.compute_vertex_normals()
    return s


def _make_path_lineset(path_zup):
    import open3d as o3d
    n = len(path_zup)
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(path_zup.astype(np.float64))
    ls.lines  = o3d.utility.Vector2iVector([[i, i + 1] for i in range(n - 1)])
    ls.paint_uniform_color([1.0, 0.1, 0.1])
    return ls


def _make_axes(size=1.0):
    import open3d as o3d
    pts = [[0,0,0],[size,0,0],[0,0,0],[0,size,0],[0,0,0],[0,0,size]]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines  = o3d.utility.Vector2iVector([[0,1],[2,3],[4,5]])
    ls.colors = o3d.utility.Vector3dVector([[1,0,0],[0,1,0],[0,0,1]])
    return ls


# ---------------------------------------------------------------------------
# Interactive picking
# ---------------------------------------------------------------------------

def _pick_two_points(nm_mesh, pcd=None):
    """
    Open a VisualizerWithEditing window.

    The user Shift+clicks the START point, then the END point, then closes
    the window (Q or the window's X button).

    Parameters
    ----------
    nm_mesh : o3d.geometry.TriangleMesh  navmesh solid mesh
    pcd     : o3d.geometry.PointCloud | None

    Returns
    -------
    (start_zup, end_zup) : tuple of (3,) float32 arrays in Z-up coords.

    Raises
    ------
    ValueError  if fewer than 2 points are picked.
    """
    import open3d as o3d

    # Build the source geometry for picking.
    # VisualizerWithEditing picks from PointCloud vertices only;
    # LineSets/TriangleMeshes are visual aids only.
    if pcd is not None and len(pcd.points) > 0:
        pick_cloud = pcd
        source_name = "point cloud"
    else:
        # Sample the navmesh surface uniformly so the user can click on it.
        pick_cloud = nm_mesh.sample_points_uniformly(number_of_points=100_000)
        pick_cloud.paint_uniform_color([0.0, 0.85, 0.35])
        source_name = "navmesh surface"

    nm_wire = _make_wireframe(nm_mesh)

    print(f"\n  ── Picking window ──────────────────────────────────────────")
    print(f"  Shift + left-click  →  pick a point on the {source_name}")
    print(f"  Pick #1 = START  (blue)   |   Pick #2 = END  (yellow)")
    print(f"  Q / close window  →  confirm and compute path")
    print(f"  ────────────────────────────────────────────────────────────\n")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(
        window_name="Shift+click START then END — Q/close to confirm",
        width=1280, height=800,
    )
    # Add pick_cloud FIRST so get_picked_points() indices map into it.
    vis.add_geometry(pick_cloud)
    vis.add_geometry(nm_wire)
    vis.run()
    vis.destroy_window()

    picked_idx = vis.get_picked_points()
    n = len(picked_idx)

    if n < 2:
        raise ValueError(
            f"Got {n} pick(s) — need exactly 2 (START then END). "
            "Shift+click two points, then close the window."
        )
    if n > 2:
        print(f"  Note: {n} points picked; using first two as START and END.")

    pts = np.asarray(pick_cloud.points, dtype=np.float32)
    return pts[picked_idx[0]], pts[picked_idx[1]]


# ---------------------------------------------------------------------------
# Path query
# ---------------------------------------------------------------------------

def _compute_path(nm, start_zup, end_zup, extents, max_nodes):
    """Run NavMeshQuery and return (path, start_snapped, end_snapped)."""
    with NavMeshQuery(nm, max_nodes=max_nodes, extents=tuple(extents)) as q:
        s_ref, s_snp = q.find_nearest_poly(start_zup)
        e_ref, e_snp = q.find_nearest_poly(end_zup)

        if s_ref == 0:
            raise ValueError(
                "Start point is not on the navmesh — try a different location "
                "or increase --extents."
            )
        if e_ref == 0:
            raise ValueError(
                "End point is not on the navmesh — try a different location "
                "or increase --extents."
            )

        print(f"  start snapped : {s_snp.round(3)}")
        print(f"  end   snapped : {e_snp.round(3)}")

        path = q.find_path(start_zup, end_zup, extents=extents)

    return path, s_snp, e_snp


# ---------------------------------------------------------------------------
# Result visualisation
# ---------------------------------------------------------------------------

def _show_result(nm_mesh, path_zup, pcd=None, start_zup=None, end_zup=None):
    """Display path, navmesh wireframe, and optional PCD."""
    import open3d as o3d

    geoms = []

    if pcd is not None and len(pcd.points) > 0:
        geoms.append(pcd)

    geoms.append(_make_wireframe(nm_mesh))

    if len(path_zup) >= 2:
        geoms.append(_make_path_lineset(path_zup))
        for pt in path_zup[1:-1]:                            # intermediate waypoints
            geoms.append(_make_sphere(pt, radius=0.12, color=[1.0, 0.4, 0.0]))

    if start_zup is not None:
        geoms.append(_make_sphere(start_zup, radius=0.25, color=[0.1, 0.4, 1.0]))   # blue
    if end_zup is not None:
        geoms.append(_make_sphere(end_zup,   radius=0.25, color=[1.0, 0.9, 0.0]))   # yellow

    ref_pts = np.asarray(pcd.points) if (pcd and len(pcd.points)) \
              else np.asarray(nm_mesh.vertices)
    geoms.append(_make_axes(size=max(float(np.ptp(ref_pts)), 1.0) * 0.05))

    title = (f"Path: {len(path_zup)} waypoints"
             if len(path_zup) else "No path found")
    o3d.visualization.draw_geometries(
        geoms, window_name=title, width=1280, height=800,
    )


# ---------------------------------------------------------------------------
# Interactive mode
# ---------------------------------------------------------------------------

def interactive_mode(nm, pcd, extents, max_nodes):
    """
    Loop:
      1. Open picking window  (Shift+click START then END)
      2. Compute path
      3. Show result window
      4. Ask "pick again?"
    """
    nm_mesh = _make_navmesh_mesh(nm)

    while True:
        # ── Pick ──────────────────────────────────────────────────────────
        try:
            start_zup, end_zup = _pick_two_points(nm_mesh, pcd)
        except ValueError as exc:
            print(f"\n  Picking error: {exc}")
            if input("  Try again? [y/N] ").strip().lower() != 'y':
                break
            continue

        print(f"\n  Picked (Z-up):")
        print(f"    START = {start_zup.round(3)}")
        print(f"    END   = {end_zup.round(3)}")

        # ── Query ─────────────────────────────────────────────────────────
        try:
            path, s_snp, e_snp = _compute_path(
                nm, start_zup, end_zup, extents, max_nodes)
        except ValueError as exc:
            print(f"\n  Query error: {exc}")
            if input("  Try again? [y/N] ").strip().lower() != 'y':
                break
            continue

        if len(path) == 0:
            print("\n  No path found between those points.")
        else:
            print(f"\n  Path: {len(path)} waypoints")
            for i, pt in enumerate(path):
                print(f"    [{i}] {pt.round(3)}")

        # ── Show ──────────────────────────────────────────────────────────
        _show_result(nm_mesh, path, pcd, s_snp, e_snp)

        if input("\n  Pick again? [y/N] ").strip().lower() != 'y':
            break


# ---------------------------------------------------------------------------
# Batch mode  (original behaviour)
# ---------------------------------------------------------------------------

def batch_mode(nm, start_zup, end_zup, extents, max_nodes, pcd_path, no_vis):
    print(f"\nQuery (Z-up):")
    print(f"  start   = {start_zup}")
    print(f"  end     = {end_zup}")
    print(f"  extents = {extents}")

    try:
        path, s_snp, e_snp = _compute_path(
            nm, start_zup, end_zup, extents, max_nodes)
    except ValueError as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        sys.exit(1)

    if len(path) == 0:
        print("\nNo path found.")
        sys.exit(1)

    print(f"\nPath: {len(path)} waypoints")
    for i, pt in enumerate(path):
        print(f"  [{i}] {pt.round(4)}")

    if not no_vis:
        import open3d as o3d
        pcd = None
        if pcd_path:
            pcd = o3d.io.read_point_cloud(pcd_path)
        nm_mesh = _make_navmesh_mesh(nm)
        _show_result(nm_mesh, path, pcd, s_snp, e_snp)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Recast/Detour path query. "
                    "Omit start/end coords to use interactive picking mode.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("bin", help="Navmesh .bin file")

    # Optional positional coords — omit all six for interactive mode
    parser.add_argument("sx", nargs="?", type=float, default=None, metavar="sx")
    parser.add_argument("sy", nargs="?", type=float, default=None, metavar="sy")
    parser.add_argument("sz", nargs="?", type=float, default=None, metavar="sz")
    parser.add_argument("ex", nargs="?", type=float, default=None, metavar="ex")
    parser.add_argument("ey", nargs="?", type=float, default=None, metavar="ey")
    parser.add_argument("ez", nargs="?", type=float, default=None, metavar="ez")

    parser.add_argument("--pcd",       default=None,
                        help="PCD file (shown in viewer; used for picking in interactive mode)")
    parser.add_argument("--extents",   nargs=3, type=float, default=[2.0, 4.0, 2.0],
                        metavar=("X", "Y", "Z"),
                        help="Poly-snap half-extents in Z-up (default: 2 4 2)")
    parser.add_argument("--max-nodes", type=int, default=2048,
                        help="A* node budget (default: 2048)")
    parser.add_argument("--no-vis",    action="store_true",
                        help="(batch only) skip Open3D visualisation")
    args = parser.parse_args()

    print(f"Loading navmesh: {args.bin}")
    nm = NavMesh(args.bin)
    print(f"  {nm.tile_count} tile(s)")

    coords = [args.sx, args.sy, args.sz, args.ex, args.ey, args.ez]
    is_batch = all(c is not None for c in coords)

    if is_batch:
        batch_mode(
            nm,
            np.array([args.sx, args.sy, args.sz], dtype=np.float32),
            np.array([args.ex, args.ey, args.ez], dtype=np.float32),
            args.extents, args.max_nodes, args.pcd, args.no_vis,
        )
    else:
        import open3d as o3d
        pcd = None
        if args.pcd:
            print(f"Loading PCD: {args.pcd}")
            pcd = o3d.io.read_point_cloud(args.pcd)
            print(f"  {len(pcd.points)} points")
        interactive_mode(nm, pcd, args.extents, args.max_nodes)

    nm.close()


if __name__ == "__main__":
    main()
