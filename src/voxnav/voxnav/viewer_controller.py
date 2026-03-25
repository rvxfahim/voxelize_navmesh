import numpy as np
import open3d as o3d


class ViewerController:
    def __init__(self):
        self._vis = None
        self._started = False
        self._geoms = {}
        self._input_mesh = None
        self._input_mesh_wire = None
        self._input_display_mode = "solid"
        self._navmesh_verts = None
        self._navmesh_tris = None
        self._navmesh_high_contrast = True

    def _ensure_started(self) -> None:
        if self._started:
            return
        self._vis = o3d.visualization.Visualizer()
        if not self._vis.create_window(window_name="Navmesh Baker Preview"):
            raise RuntimeError("Failed to create Open3D preview window.")
        self._started = True

    def close(self) -> None:
        if self._started and self._vis is not None:
            self._vis.destroy_window()
            self._started = False

    def poll(self) -> bool:
        if not self._started or self._vis is None:
            return True
        ok = self._vis.poll_events()
        self._vis.update_renderer()
        return ok

    def set_input_mesh(self, obj_path: str) -> None:
        self._ensure_started()
        mesh = o3d.io.read_triangle_mesh(obj_path)
        if mesh.is_empty():
            raise RuntimeError(f"Failed to load OBJ: {obj_path}")
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        mesh.paint_uniform_color((0.55, 0.55, 0.55))
        self._input_mesh = mesh
        self._input_mesh_wire = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
        self._input_mesh_wire.paint_uniform_color((0.75, 0.75, 0.75))
        self._refresh_input_mesh_geometry()

    def set_navmesh_wireframe(self, verts: np.ndarray, tris: np.ndarray) -> None:
        self._ensure_started()
        self._navmesh_verts = verts
        self._navmesh_tris = tris
        self._refresh_navmesh_geometry()

    def set_input_mesh_display_mode(self, mode: str) -> None:
        if mode not in ("solid", "wireframe", "hidden"):
            raise ValueError(f"Unsupported input mesh display mode: {mode}")
        self._input_display_mode = mode
        self._refresh_input_mesh_geometry()

    def set_navmesh_high_contrast(self, enabled: bool) -> None:
        self._navmesh_high_contrast = bool(enabled)
        self._refresh_navmesh_geometry()

    def _refresh_navmesh_geometry(self) -> None:
        if self._navmesh_verts is None or self._navmesh_tris is None:
            return
        tri = o3d.geometry.TriangleMesh()
        tri.vertices = o3d.utility.Vector3dVector(self._navmesh_verts.astype(np.float64))
        tri.triangles = o3d.utility.Vector3iVector(self._navmesh_tris.astype(np.int32))
        lines = o3d.geometry.LineSet.create_from_triangle_mesh(tri)
        if self._navmesh_high_contrast:
            lines.paint_uniform_color((1.0, 0.2, 0.0))
        else:
            lines.paint_uniform_color((0.0, 0.8, 0.2))
        self._set_geometry("navmesh_wire", lines)

    def _refresh_input_mesh_geometry(self) -> None:
        if self._input_mesh is None:
            self._remove_geometry("input_mesh")
            self._remove_geometry("input_mesh_wire")
            return
        if self._input_display_mode == "hidden":
            self._remove_geometry("input_mesh")
            self._remove_geometry("input_mesh_wire")
        elif self._input_display_mode == "wireframe":
            self._remove_geometry("input_mesh")
            self._set_geometry("input_mesh_wire", self._input_mesh_wire)
        else:
            self._remove_geometry("input_mesh_wire")
            self._set_geometry("input_mesh", self._input_mesh)

    def _set_geometry(self, key: str, geom) -> None:
        if not self._started or self._vis is None:
            return
        if key in self._geoms:
            self._vis.remove_geometry(self._geoms[key], reset_bounding_box=False)
        self._geoms[key] = geom
        self._vis.add_geometry(geom, reset_bounding_box=(len(self._geoms) == 1))

    def _remove_geometry(self, key: str) -> None:
        if not self._started or self._vis is None:
            return
        if key in self._geoms:
            self._vis.remove_geometry(self._geoms[key], reset_bounding_box=False)
            del self._geoms[key]
