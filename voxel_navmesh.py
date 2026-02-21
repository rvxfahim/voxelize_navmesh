"""
Voxelize Point Cloud → Mesh → Recast Navmesh Pipeline

Loads a PCD file, voxelizes it, generates a triangle mesh via exposed-face
culling, exports OBJ, then uses Recast Navigation (via recast_cli) for
navmesh generation and Detour pathfinding.
"""

import json
import os
import subprocess
import time

import numpy as np
import open3d as o3d
import trimesh


RECAST_CLI = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "build", "recast_cli")

# Axis remapping: source_up -> (column indices to reorder XYZ so Y is up)
_UP_AXIS_REMAP = {
    "y": None,                 # already Y-up, no transform
    "z": [0, 2, 1],           # Z-up → Y-up: (x,y,z) → (x,z,y)
    "x": [1, 0, 2],           # X-up → Y-up: (x,y,z) → (y,x,z)
}


def reorient_to_yup(points, source_up="z"):
    """Reorder point cloud axes so that Y is up.

    Parameters
    ----------
    points : ndarray (N, 3)
    source_up : str
        Which axis is "up" in the source data: 'x', 'y', or 'z'.

    Returns
    -------
    ndarray (N, 3)  with Y as the up axis.
    """
    source_up = source_up.lower()
    if source_up not in _UP_AXIS_REMAP:
        raise ValueError(f"Unknown up axis '{source_up}', expected x/y/z")
    remap = _UP_AXIS_REMAP[source_up]
    if remap is None:
        return points
    reoriented = points[:, remap].copy()
    print(f"  Reoriented points: {source_up.upper()}-up → Y-up")
    return reoriented


# ---------------------------------------------------------------------------
# Step 1: Load PCD
# ---------------------------------------------------------------------------
def load_pcd(path):
    print(f"Loading PCD from {path} ...")
    pcd = o3d.io.read_point_cloud(path)
    points = np.asarray(pcd.points)
    print(f"  Loaded {len(points):,} points")
    return pcd, points


# ---------------------------------------------------------------------------
# Step 2: Voxelize
# ---------------------------------------------------------------------------
def voxelize(points, voxel_size=None):
    bb_min = points.min(axis=0)
    bb_max = points.max(axis=0)
    extent = bb_max - bb_min

    if voxel_size is None:
        voxel_size = extent.max() / 200.0
    print(f"  Voxel size: {voxel_size:.4f}")
    print(f"  Bounding box extent: {extent}")

    # Quantize points to grid indices
    indices = np.floor((points - bb_min) / voxel_size).astype(np.int32)
    occupied = set(map(tuple, indices))
    print(f"  Occupied voxels: {len(occupied):,}")
    return occupied, bb_min, voxel_size


# ---------------------------------------------------------------------------
# Step 3: Voxel → Triangle Mesh (exposed-face culling)
# ---------------------------------------------------------------------------

# Six face directions and their quad vertex offsets (CCW winding)
_FACE_DEFS = {
    # direction : (normal_dir, 4 corner offsets)
    ( 1,  0,  0): [(1,0,0), (1,1,0), (1,1,1), (1,0,1)],  # +X
    (-1,  0,  0): [(0,0,1), (0,1,1), (0,1,0), (0,0,0)],  # -X
    ( 0,  1,  0): [(0,1,0), (0,1,1), (1,1,1), (1,1,0)],  # +Y
    ( 0, -1,  0): [(1,0,0), (1,0,1), (0,0,1), (0,0,0)],  # -Y
    ( 0,  0,  1): [(0,0,1), (1,0,1), (1,1,1), (0,1,1)],  # +Z
    ( 0,  0, -1): [(0,0,0), (0,1,0), (1,1,0), (1,0,0)],  # -Z
}


def voxels_to_mesh(occupied, bb_min, voxel_size):
    print("Generating triangle mesh from voxels ...")
    t0 = time.time()

    vertex_map = {}  # (vx, vy, vz) -> index
    vertices = []
    faces = []

    def get_vertex(v):
        key = v
        if key not in vertex_map:
            vertex_map[key] = len(vertices)
            vertices.append(v)
        return vertex_map[key]

    for (ix, iy, iz) in occupied:
        for (dx, dy, dz), corners in _FACE_DEFS.items():
            neighbour = (ix + dx, iy + dy, iz + dz)
            if neighbour in occupied:
                continue  # face hidden by adjacent voxel
            # Emit quad as 2 triangles
            quad = []
            for (cx, cy, cz) in corners:
                world = (
                    bb_min[0] + (ix + cx) * voxel_size,
                    bb_min[1] + (iy + cy) * voxel_size,
                    bb_min[2] + (iz + cz) * voxel_size,
                )
                quad.append(get_vertex(world))
            faces.append([quad[0], quad[1], quad[2]])
            faces.append([quad[0], quad[2], quad[3]])

    verts_arr = np.array(vertices, dtype=np.float64)
    faces_arr = np.array(faces, dtype=np.int64)
    elapsed = time.time() - t0
    print(f"  Vertices: {len(verts_arr):,}  Faces: {len(faces_arr):,}  ({elapsed:.1f}s)")
    return verts_arr, faces_arr


def voxels_to_mesh_greedy(occupied, bb_min, voxel_size):
    """Greedy meshing: merge coplanar adjacent faces into maximal rectangles."""
    print("Generating triangle mesh from voxels (greedy meshing) ...")
    t0 = time.time()

    vertex_map = {}
    vertices = []
    faces_out = []

    def get_vertex(v):
        if v not in vertex_map:
            vertex_map[v] = len(vertices)
            vertices.append(v)
        return vertex_map[v]

    # For each axis (0=X, 1=Y, 2=Z) and direction (+1, -1)
    for axis in range(3):
        for direction in (+1, -1):
            # Build a dict: slice_coord -> set of (u, v) positions with exposed faces
            slices = {}
            for voxel in occupied:
                neighbour = list(voxel)
                neighbour[axis] += direction
                if tuple(neighbour) in occupied:
                    continue  # face hidden
                # slice coordinate along this axis
                s = voxel[axis] + (1 if direction == 1 else 0)
                # u, v are the two axes perpendicular to `axis`
                axes_uv = [a for a in range(3) if a != axis]
                u_idx, v_idx = axes_uv
                uv = (voxel[u_idx], voxel[v_idx])
                slices.setdefault(s, set()).add(uv)

            axes_uv = [a for a in range(3) if a != axis]
            u_idx, v_idx = axes_uv

            # Greedy merge each slice
            for s, face_set in slices.items():
                visited = set()
                for (u, v) in sorted(face_set):
                    if (u, v) in visited:
                        continue
                    # Expand width (u direction)
                    w = 1
                    while (u + w, v) in face_set and (u + w, v) not in visited:
                        w += 1
                    # Expand height (v direction)
                    h = 1
                    done = False
                    while not done:
                        for du in range(w):
                            if (u + du, v + h) not in face_set or (u + du, v + h) in visited:
                                done = True
                                break
                        if not done:
                            h += 1

                    # Mark visited
                    for du in range(w):
                        for dv in range(h):
                            visited.add((u + du, v + dv))

                    # Build quad corners in world coords
                    # The four corners of the merged rectangle
                    corners_uv = [
                        (u, v),
                        (u + w, v),
                        (u + w, v + h),
                        (u, v + h),
                    ]

                    quad = []
                    for (cu, cv) in corners_uv:
                        pt = [0.0, 0.0, 0.0]
                        pt[axis] = bb_min[axis] + s * voxel_size
                        pt[u_idx] = bb_min[u_idx] + cu * voxel_size
                        pt[v_idx] = bb_min[v_idx] + cv * voxel_size
                        quad.append(get_vertex(tuple(pt)))

                    # Winding order depends on direction
                    if direction == 1:
                        faces_out.append([quad[0], quad[1], quad[2]])
                        faces_out.append([quad[0], quad[2], quad[3]])
                    else:
                        faces_out.append([quad[0], quad[2], quad[1]])
                        faces_out.append([quad[0], quad[3], quad[2]])

    verts_arr = np.array(vertices, dtype=np.float64)
    faces_arr = np.array(faces_out, dtype=np.int64)
    elapsed = time.time() - t0
    print(f"  Vertices: {len(verts_arr):,}  Faces: {len(faces_arr):,}  ({elapsed:.1f}s)")
    return verts_arr, faces_arr


def voxels_to_mesh_marching_cubes(occupied, bb_min, voxel_size):
    """Marching cubes: extract a smooth isosurface from the voxel volume."""
    from skimage.measure import marching_cubes

    print("Generating triangle mesh from voxels (marching cubes) ...")
    t0 = time.time()

    # Build 3D boolean volume
    occ_arr = np.array(list(occupied))
    grid_min = occ_arr.min(axis=0)
    grid_max = occ_arr.max(axis=0)
    shape = (grid_max - grid_min) + 3  # +3 for 1-voxel padding on each side
    volume = np.zeros(shape, dtype=np.float32)
    shifted = occ_arr - grid_min + 1  # +1 for padding
    volume[shifted[:, 0], shifted[:, 1], shifted[:, 2]] = 1.0

    verts_grid, faces_mc, _, _ = marching_cubes(volume, level=0.5)

    # Transform back to world coords
    verts_world = (verts_grid - 1 + grid_min) * voxel_size + bb_min

    verts_arr = np.asarray(verts_world, dtype=np.float64)
    faces_arr = np.asarray(faces_mc, dtype=np.int64)
    elapsed = time.time() - t0
    print(f"  Vertices: {len(verts_arr):,}  Faces: {len(faces_arr):,}  ({elapsed:.1f}s)")
    return verts_arr, faces_arr


def export_obj(verts, faces, path, swap_yz=True):
    if swap_yz:
        # Swap Y↔Z so Recast (Y-up) sees the correct floor orientation.
        # Our model is Z-up (XY floor), Recast expects Y-up (XZ floor).
        out_verts = verts.copy()
        out_verts[:, 1], out_verts[:, 2] = verts[:, 2].copy(), verts[:, 1].copy()
        # Reverse face winding to fix normals after axis swap (handedness change)
        out_faces = faces[:, ::-1].copy()
        label = "(Y↔Z swapped for Recast)"
    else:
        out_verts = verts
        out_faces = faces
        label = "(no axis swap)"
    mesh = trimesh.Trimesh(vertices=out_verts, faces=out_faces)
    mesh.export(path)
    print(f"  Exported mesh to {path} {label}")


# ---------------------------------------------------------------------------
# Step 4: Recast Navmesh + Detour Pathfinding (via CLI)
# ---------------------------------------------------------------------------
def run_recast_pathfinding(obj_path, start, end):
    """Call recast_cli to build navmesh and find path.

    Returns list of [x,y,z] waypoints, or None if no path found.
    """
    # Swap Y↔Z to match the exported OBJ (Recast is Y-up)
    cmd = [
        RECAST_CLI, obj_path,
        str(start[0]), str(start[2]), str(start[1]),
        str(end[0]), str(end[2]), str(end[1]),
    ]
    print(f"  Running: {' '.join(cmd)}")
    t0 = time.time()
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    elapsed = time.time() - t0

    # Print stderr (Recast diagnostic messages)
    if result.stderr:
        for line in result.stderr.strip().split("\n"):
            print(f"  [recast] {line}")

    if result.returncode != 0:
        print(f"  recast_cli failed (exit {result.returncode})")
        return None

    data = json.loads(result.stdout)
    if data["status"] != "ok":
        print(f"  Recast error: {data.get('message', 'unknown')}")
        return None

    # Swap Y↔Z back to our Z-up coordinate system
    path = [[p[0], p[2], p[1]] for p in data["path"]]
    nm = data["navmesh"]
    print(f"  Navmesh: {nm['verts']} verts, {nm['polys']} polys")
    print(f"  Path: {len(path)} waypoints ({elapsed:.2f}s)")
    return path


# ---------------------------------------------------------------------------
# Step 5: Interactive Point Picking + Visualization
# ---------------------------------------------------------------------------
def pick_start_end(verts, faces):
    """Open a viewer for the user to Shift+Click two points on the mesh."""
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.7, 0.7, 0.7])

    # Create a point cloud from mesh vertices for picking
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(verts)
    pcd.paint_uniform_color([0.1, 0.8, 0.2])

    # Convert mesh to wireframe so it doesn't block picking
    wireframe = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    wireframe.paint_uniform_color([0.6, 0.6, 0.6])

    print("\n=== Point Picker ===")
    print("  1. Hold Shift + Left-Click to pick the START point")
    print("  2. Hold Shift + Left-Click to pick the END point")
    print("  3. Close the window (Q or X) when done\n")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Pick START and END points  [Shift+Click]",
                      width=1280, height=720)
    # Point cloud FIRST — this is what gets picked
    vis.add_geometry(pcd)
    vis.add_geometry(wireframe)
    vis.get_render_option().point_size = 5.0
    vis.run()
    vis.destroy_window()

    picked_indices = vis.get_picked_points()
    if len(picked_indices) < 2:
        print(f"  Only {len(picked_indices)} point(s) picked — need at least 2.")
        return None, None

    start_world = np.array(verts[picked_indices[0]])
    end_world = np.array(verts[picked_indices[1]])
    print(f"  Picked start (world): {start_world}")
    print(f"  Picked end   (world): {end_world}")
    return start_world, end_world


def visualize(verts, faces, path_waypoints=None, start_world=None, end_world=None,
              voxel_size=0.1):
    geometries = []

    # Voxel mesh (light grey)
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(faces)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.7, 0.7, 0.7])
    geometries.append(mesh)

    # Start / End markers (spheres)
    for pt, color in [(start_world, [0, 0, 1]), (end_world, [1, 0.5, 0])]:
        if pt is not None:
            sphere = o3d.geometry.TriangleMesh.create_sphere(
                radius=voxel_size * 2.0)
            sphere.translate(pt)
            sphere.paint_uniform_color(color)
            sphere.compute_vertex_normals()
            geometries.append(sphere)

    # Navigation path (red line)
    if path_waypoints and len(path_waypoints) > 1:
        path_pts = [[p[0], p[1], p[2]] for p in path_waypoints]
        lines = [[i, i + 1] for i in range(len(path_pts) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(path_pts)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(
            [[1, 0, 0]] * len(lines)
        )
        geometries.append(line_set)

    print("Opening result visualization ...")
    o3d.visualization.draw_geometries(
        geometries,
        window_name="Voxel Navmesh — Path Result (Recast)",
        width=1280,
        height=720,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
MESH_METHODS = {
    "naive": voxels_to_mesh,
    "greedy": voxels_to_mesh_greedy,
    "marching_cubes": voxels_to_mesh_marching_cubes,
}


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Voxel navmesh pipeline")
    parser.add_argument("--method", choices=MESH_METHODS.keys(), default="greedy",
                        help="Meshing method (default: greedy)")
    parser.add_argument("--up-axis", choices=["x", "y", "z"], default=None,
                        help="Source up axis of the PCD (x/y/z). "
                             "Points are reoriented to Y-up before voxelization. "
                             "Implies no axis swap at export.")
    parser.add_argument("--no-swap-yz", action="store_true",
                        help="Disable Y↔Z axis swap in exported OBJ")
    args = parser.parse_args()

    pcd_path = "fhparking.pcd"
    obj_path = "voxel_mesh.obj"

    # 1. Load
    _, points = load_pcd(pcd_path)

    # Pre-process: reorient so Y is up before voxelization
    if args.up_axis:
        points = reorient_to_yup(points, args.up_axis)
        swap_yz = False
    else:
        swap_yz = not args.no_swap_yz

    # 2. Voxelize
    occupied, bb_min, voxel_size = voxelize(points)

    # 3. Mesh
    mesh_fn = MESH_METHODS[args.method]
    verts, faces = mesh_fn(occupied, bb_min, voxel_size)
    export_obj(verts, faces, obj_path, swap_yz=swap_yz)

    # 4. Interactive point picking
    start_world, end_world = pick_start_end(verts, faces)

    path_waypoints = None
    if start_world is not None and end_world is not None:
        # 5. Recast navmesh + Detour pathfinding
        print("Running Recast pathfinding ...")
        path_waypoints = run_recast_pathfinding(obj_path, start_world, end_world)
        if path_waypoints is None:
            print("  No path found.")
    else:
        print("  Skipping pathfinding (no points selected)")

    # 6. Visualize result
    visualize(verts, faces, path_waypoints, start_world, end_world, voxel_size)


if __name__ == "__main__":
    main()
