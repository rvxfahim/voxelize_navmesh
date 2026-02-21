"""
navmesh.py — Python ctypes wrapper over navmesh_bridge.so (Detour dtNavMesh).

All public-facing coordinates are in the caller's Z-up frame (matching the
original point cloud / Open3D convention).  The Y↔Z axis swap to/from Recast's
Y-up convention is handled transparently inside this module.

Usage
-----
    from navmesh import NavMesh, NavMeshQuery

    nm = NavMesh("scene.bin")
    verts, tris = nm.get_geometry()   # numpy (N,3) and (M,3), Z-up

    with NavMeshQuery(nm) as q:
        path = q.find_path(start_zup, end_zup)   # numpy (K,3), Z-up

Build the shared library first:
    cmake --build build --target navmesh_bridge
"""

import ctypes
import os
import numpy as np

# ---------------------------------------------------------------------------
# Locate the shared library relative to this file
# ---------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_LIB_SEARCH = [
    os.path.join(_HERE, "build", "navmesh_bridge.so"),
    os.path.join(_HERE, "navmesh_bridge.so"),
]

def _load_lib():
    for p in _LIB_SEARCH:
        if os.path.exists(p):
            return ctypes.CDLL(p)
    raise FileNotFoundError(
        "navmesh_bridge.so not found. Build it with:\n"
        "  cmake --build build --target navmesh_bridge"
    )

_lib = _load_lib()

# ---------------------------------------------------------------------------
# ctypes signatures
# ---------------------------------------------------------------------------
_lib.nm_load.restype              = ctypes.c_void_p
_lib.nm_load.argtypes             = [ctypes.c_char_p]

_lib.nm_free.restype              = None
_lib.nm_free.argtypes             = [ctypes.c_void_p]

_lib.nm_tile_count.restype        = ctypes.c_int
_lib.nm_tile_count.argtypes       = [ctypes.c_void_p]

_lib.nm_tile_bounds.restype       = ctypes.c_int
_lib.nm_tile_bounds.argtypes      = [ctypes.c_void_p, ctypes.c_int,
                                     ctypes.POINTER(ctypes.c_float),
                                     ctypes.POINTER(ctypes.c_float)]

_lib.nm_tile_detail_verts.restype = ctypes.c_int
_lib.nm_tile_detail_verts.argtypes= [ctypes.c_void_p, ctypes.c_int,
                                     ctypes.POINTER(ctypes.c_float),
                                     ctypes.c_int]

_lib.nm_tile_detail_tris.restype  = ctypes.c_int
_lib.nm_tile_detail_tris.argtypes = [ctypes.c_void_p, ctypes.c_int,
                                     ctypes.POINTER(ctypes.c_int),
                                     ctypes.c_int]

_lib.nm_query_create.restype      = ctypes.c_void_p
_lib.nm_query_create.argtypes     = [ctypes.c_void_p, ctypes.c_int]

_lib.nm_query_free.restype        = None
_lib.nm_query_free.argtypes       = [ctypes.c_void_p]

_lib.nm_find_nearest_poly.restype = ctypes.c_uint32
_lib.nm_find_nearest_poly.argtypes= [ctypes.c_void_p,
                                     ctypes.POINTER(ctypes.c_float),
                                     ctypes.POINTER(ctypes.c_float),
                                     ctypes.POINTER(ctypes.c_float)]

_lib.nm_find_path.restype         = ctypes.c_int
_lib.nm_find_path.argtypes        = [ctypes.c_void_p,
                                     ctypes.c_uint32, ctypes.c_uint32,
                                     ctypes.POINTER(ctypes.c_float),
                                     ctypes.POINTER(ctypes.c_float),
                                     ctypes.POINTER(ctypes.c_uint32),
                                     ctypes.c_int]

_lib.nm_find_straight_path.restype = ctypes.c_int
_lib.nm_find_straight_path.argtypes= [ctypes.c_void_p,
                                      ctypes.POINTER(ctypes.c_float),
                                      ctypes.POINTER(ctypes.c_float),
                                      ctypes.POINTER(ctypes.c_uint32),
                                      ctypes.c_int,
                                      ctypes.POINTER(ctypes.c_float),
                                      ctypes.c_int]

# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------
def _to_yup(xyz_zup: np.ndarray) -> np.ndarray:
    """(N,3) Z-up  →  (N,3) Y-up:  (x, y, z) → (x, z, y)"""
    out = xyz_zup.copy()
    out[:, 1] = xyz_zup[:, 2]
    out[:, 2] = xyz_zup[:, 1]
    return out

def _to_zup(xyz_yup: np.ndarray) -> np.ndarray:
    """(N,3) Y-up  →  (N,3) Z-up:  (x, y, z) → (x, z, y)"""
    return _to_yup(xyz_yup)  # same swap

def _vec3f(*args):
    """Build a ctypes c_float[3] from 3 floats or a 3-element sequence."""
    if len(args) == 1:
        v = args[0]
        return (ctypes.c_float * 3)(float(v[0]), float(v[1]), float(v[2]))
    return (ctypes.c_float * 3)(float(args[0]), float(args[1]), float(args[2]))


# ---------------------------------------------------------------------------
# NavMesh
# ---------------------------------------------------------------------------
class NavMesh:
    """
    Loads a Recast/Detour navmesh from a .bin file saved by RecastDemo
    (or by recast_cli --save-bin).

    Parameters
    ----------
    bin_path : str
        Path to the .bin navmesh file.
    """

    def __init__(self, bin_path: str):
        self._path = bin_path
        self._handle = _lib.nm_load(bin_path.encode())
        if not self._handle:
            raise RuntimeError(f"Failed to load navmesh from '{bin_path}'. "
                               "Check the file is a valid RecastDemo .bin.")

    # ------------------------------------------------------------------
    def close(self):
        if self._handle:
            _lib.nm_free(ctypes.c_void_p(self._handle))
            self._handle = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    @property
    def tile_count(self) -> int:
        """Number of valid tiles in the navmesh."""
        return _lib.nm_tile_count(ctypes.c_void_p(self._handle))

    def tile_bounds(self, tile_idx: int):
        """
        Returns (bmin, bmax) in Z-up coordinates, each a (3,) float array.
        """
        bmin_c = (ctypes.c_float * 3)()
        bmax_c = (ctypes.c_float * 3)()
        rc = _lib.nm_tile_bounds(ctypes.c_void_p(self._handle), tile_idx,
                                 bmin_c, bmax_c)
        if rc < 0:
            raise IndexError(f"tile_idx {tile_idx} out of range")
        bmin_yup = np.array(list(bmin_c), dtype=np.float32).reshape(1, 3)
        bmax_yup = np.array(list(bmax_c), dtype=np.float32).reshape(1, 3)
        return _to_zup(bmin_yup)[0], _to_zup(bmax_yup)[0]

    # ------------------------------------------------------------------
    def get_geometry(self, swap_yz: bool = True):
        """
        Extract the navmesh detail-mesh geometry across all tiles.

        Parameters
        ----------
        swap_yz : bool
            True (default) → output in Z-up frame (matches PCD / Open3D).
            False → raw Y-up Recast coordinates.

        Returns
        -------
        verts : np.ndarray, shape (N, 3), float32
        tris  : np.ndarray, shape (M, 3), int32
        """
        all_verts = []
        all_tris  = []
        vert_offset = 0

        for ti in range(self.tile_count):
            # --- vertices ---
            nf = _lib.nm_tile_detail_verts(
                ctypes.c_void_p(self._handle), ti, None, 0)
            if nf <= 0:
                continue
            vbuf = (ctypes.c_float * nf)()
            _lib.nm_tile_detail_verts(
                ctypes.c_void_p(self._handle), ti, vbuf, nf)
            verts_yup = np.frombuffer(vbuf, dtype=np.float32).reshape(-1, 3).copy()

            # --- triangles ---
            ni = _lib.nm_tile_detail_tris(
                ctypes.c_void_p(self._handle), ti, None, 0)
            if ni <= 0:
                continue
            ibuf = (ctypes.c_int * ni)()
            written = _lib.nm_tile_detail_tris(
                ctypes.c_void_p(self._handle), ti, ibuf, ni)
            tris = np.frombuffer(ibuf, dtype=np.int32)[:written].reshape(-1, 3).copy()

            all_verts.append(verts_yup)
            all_tris.append(tris + vert_offset)
            vert_offset += len(verts_yup)

        if not all_verts:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)

        verts = np.concatenate(all_verts, axis=0)
        tris  = np.concatenate(all_tris,  axis=0)

        if swap_yz:
            verts = _to_zup(verts)

        return verts.astype(np.float32), tris.astype(np.int32)

    # ------------------------------------------------------------------
    def _raw_handle(self):
        return self._handle


# ---------------------------------------------------------------------------
# NavMeshQuery
# ---------------------------------------------------------------------------
class NavMeshQuery:
    """
    Wraps dtNavMeshQuery.  All coordinates are in Z-up (PCD) space.

    Parameters
    ----------
    navmesh   : NavMesh
    max_nodes : int   A* node budget (higher = more accurate, slower).
    extents   : sequence of 3 floats
        Search half-extents in Z-up space for snapping points to polygons.
        Default is 2 m in X/Z and 4 m in Y (up).
    max_path  : int   Maximum number of polygon-corridor hops.
    """

    def __init__(self, navmesh: NavMesh,
                 max_nodes: int = 2048,
                 extents=(2.0, 4.0, 2.0),
                 max_path: int = 1024):
        self._nm  = navmesh
        self._ext = extents
        self._max_path = max_path

        self._handle = _lib.nm_query_create(
            ctypes.c_void_p(navmesh._raw_handle()), max_nodes)
        if not self._handle:
            raise RuntimeError("Failed to create NavMeshQuery")

    # ------------------------------------------------------------------
    def close(self):
        if self._handle:
            _lib.nm_query_free(ctypes.c_void_p(self._handle))
            self._handle = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    def _to_c3(self, pt_zup):
        """Convert a Z-up point (3,) to Y-up ctypes c_float[3]."""
        arr = np.asarray(pt_zup, dtype=np.float32).reshape(1, 3)
        yup = _to_yup(arr)[0]
        return _vec3f(yup[0], yup[1], yup[2])

    def _ext_c3(self):
        """Search extents: Z-up (x, y_zup=height, z) → Y-up (x, z, y)."""
        ex, ey_zup, ez = self._ext
        # In Z-up: ext[1] is the vertical extent; in Y-up that maps to Y
        return _vec3f(ex, ey_zup, ez)

    # ------------------------------------------------------------------
    def find_nearest_poly(self, pos_zup, extents=None):
        """
        Snap a Z-up position to the nearest navmesh polygon.

        Returns
        -------
        poly_ref : int   (0 = not found)
        snapped  : np.ndarray (3,) float32, Z-up
        """
        pos_c  = self._to_c3(pos_zup)
        ext_c  = (ctypes.c_float * 3)(*(extents or self._ext))
        snp_c  = (ctypes.c_float * 3)()

        ref = _lib.nm_find_nearest_poly(
            ctypes.c_void_p(self._handle), pos_c, ext_c, snp_c)

        snp_yup = np.array(list(snp_c), dtype=np.float32).reshape(1, 3)
        snp_zup = _to_zup(snp_yup)[0]
        return int(ref), snp_zup

    # ------------------------------------------------------------------
    def find_path(self, start_zup, end_zup, extents=None):
        """
        Find a straight-line-smoothed path between two Z-up positions.

        Parameters
        ----------
        start_zup, end_zup : array-like (3,)

        Returns
        -------
        waypoints : np.ndarray (K, 3) float32, Z-up coordinates.
                    Empty if no path found.
        """
        start_ref, start_snp = self.find_nearest_poly(start_zup, extents)
        end_ref,   end_snp   = self.find_nearest_poly(end_zup,   extents)

        if start_ref == 0:
            raise ValueError("start point is not on the navmesh")
        if end_ref == 0:
            raise ValueError("end point is not on the navmesh")

        # Convert snapped points back to Y-up for the Detour API
        s_yup = _to_yup(start_snp.reshape(1, 3))[0]
        e_yup = _to_yup(end_snp.reshape(1, 3))[0]
        start_c = _vec3f(*s_yup)
        end_c   = _vec3f(*e_yup)

        # Polygon corridor
        polys = (ctypes.c_uint32 * self._max_path)()
        npolys = _lib.nm_find_path(
            ctypes.c_void_p(self._handle),
            ctypes.c_uint32(start_ref), ctypes.c_uint32(end_ref),
            start_c, end_c, polys, self._max_path)

        if npolys <= 0:
            return np.zeros((0, 3), dtype=np.float32)

        # Straight path
        path_buf = (ctypes.c_float * (self._max_path * 3))()
        npts = _lib.nm_find_straight_path(
            ctypes.c_void_p(self._handle),
            start_c, end_c, polys, npolys,
            path_buf, self._max_path)

        if npts <= 0:
            return np.zeros((0, 3), dtype=np.float32)

        pts_yup = np.frombuffer(path_buf, dtype=np.float32)[:npts * 3] \
                    .reshape(-1, 3).copy()
        return _to_zup(pts_yup).astype(np.float32)
