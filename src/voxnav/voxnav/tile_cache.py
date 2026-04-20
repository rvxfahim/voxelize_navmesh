"""Python wrapper for the dtTileCache-based navmesh system."""

import ctypes
import numpy as np

from ._coords import to_yup, vec3f
from .navmesh_core import NavMeshRaw


class TileCache:
    """Manages a tiled dtNavMesh with a dtTileCache for dynamic cylinder obstacles.

    Construct from a .tcbin file (fast) or bake fresh from an OBJ (slow, one-time):

        tc = TileCache(tcbin_path="scene.tcbin")
        tc = TileCache(obj_path="scene_yup.obj", settings=s, tile_size=32, max_obstacles=128)

    All position coordinates are Z-up (x right, y forward, z up) to match the
    rest of the Python API. Internal Y-up conversion is done transparently.
    """

    def __init__(self, lib, bound, *, tcbin_path=None, obj_path=None,
                 settings=None, tile_size=32, max_obstacles=256,
                 input_is_z_up=True):
        self._lib = lib
        self._handle = None  # TileCacheHandle*
        self._navmesh_raw = None  # NavMeshRaw wrapping the inner dtNavMesh*

        if tcbin_path is not None:
            self._handle = lib.nm_tc_load(tcbin_path.encode())
            if not self._handle:
                raise RuntimeError(f"Failed to load tile cache from '{tcbin_path}'.")
        elif obj_path is not None:
            if settings is None:
                raise ValueError("settings must be provided when baking from OBJ.")
            err_buf = ctypes.create_string_buffer(512)
            self._handle = lib.nm_build_tiled_with_cache(
                obj_path.encode(),
                ctypes.byref(settings),
                int(input_is_z_up),
                int(tile_size),
                int(max_obstacles),
                err_buf,
                512,
            )
            if not self._handle:
                raise RuntimeError(
                    f"nm_build_tiled_with_cache failed: {err_buf.value.decode()}"
                )
        else:
            raise ValueError("Provide either tcbin_path or obj_path.")

        raw_nm_ptr = lib.nm_tc_get_navmesh(ctypes.c_void_p(self._handle))
        if not raw_nm_ptr:
            self.close()
            raise RuntimeError("nm_tc_get_navmesh returned null.")
        self._navmesh_raw = NavMeshRaw(raw_nm_ptr, lib)

    @property
    def navmesh(self) -> NavMeshRaw:
        """The inner dtNavMesh* as a non-owning NavMeshRaw wrapper."""
        return self._navmesh_raw

    def save(self, path: str) -> None:
        """Save the tile cache (compressed tile blobs + params) to a .tcbin file."""
        if self._lib.nm_tc_save(ctypes.c_void_p(self._handle), path.encode()) != 0:
            raise RuntimeError(f"Failed to save tile cache to '{path}'.")

    def add_cylinder(self, pos_zup, radius: float, height: float) -> int:
        """Insert a cylinder obstacle centred at pos_zup (Z-up).

        Returns the obstacle ref (>0) on success, 0 if the pool is full.
        The cylinder is centred at pos_zup; pos_zup[2] is the vertical centre.
        """
        # Convert centre to Y-up, then lower pos_yup[1] to the base of the cylinder.
        p = np.asarray(pos_zup, dtype=np.float32).reshape(1, 3)
        p_yup = to_yup(p)[0]
        p_yup[1] -= height / 2.0  # pos passed to Detour is the bottom of the cylinder
        pos_c = vec3f(p_yup[0], p_yup[1], p_yup[2])
        ref = self._lib.nm_tc_add_cylinder(
            ctypes.c_void_p(self._handle),
            ctypes.cast(pos_c, ctypes.POINTER(ctypes.c_float)),
            ctypes.c_float(radius),
            ctypes.c_float(height),
        )
        return int(ref)

    def remove_obstacle(self, ref: int) -> None:
        """Remove an obstacle by its ref. No-op if ref is 0 or already removed."""
        if ref:
            self._lib.nm_tc_remove_obstacle(ctypes.c_void_p(self._handle),
                                             ctypes.c_uint32(ref))

    def update(self, dt: float) -> bool:
        """Process one round of pending tile rebuilds.

        Returns True when all pending work is complete (no more tiles to rebuild).
        Call in a loop until True to fully drain the rebuild queue.
        """
        done = ctypes.c_int(0)
        self._lib.nm_tc_update(
            ctypes.c_void_p(self._handle),
            ctypes.c_float(dt),
            ctypes.byref(done),
        )
        return bool(done.value)

    @property
    def tile_count(self) -> int:
        return self._lib.nm_tc_tile_count(ctypes.c_void_p(self._handle))

    def close(self) -> None:
        if self._handle:
            self._lib.nm_tc_free(ctypes.c_void_p(self._handle))
            self._handle = None
        self._navmesh_raw = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
