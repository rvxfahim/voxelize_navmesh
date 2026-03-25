import ctypes
import numpy as np

from ._coords import to_yup, to_zup, vec3f


class NavMeshQuery:
    def __init__(self, navmesh, lib, max_nodes: int = 2048, extents=(2.0, 4.0, 2.0), max_path: int = 1024):
        self._nm = navmesh
        self._lib = lib
        self._ext = extents
        self._max_path = max_path

        self._handle = self._lib.nm_query_create(ctypes.c_void_p(navmesh._raw_handle()), max_nodes)
        if not self._handle:
            raise RuntimeError("Failed to create NavMeshQuery")

    def close(self):
        if self._handle:
            self._lib.nm_query_free(ctypes.c_void_p(self._handle))
            self._handle = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _to_c3(self, pt_zup):
        arr = np.asarray(pt_zup, dtype=np.float32).reshape(1, 3)
        yup = to_yup(arr)[0]
        return vec3f(yup[0], yup[1], yup[2])

    def find_nearest_poly(self, pos_zup, extents=None):
        pos_c = self._to_c3(pos_zup)
        ext_c = (ctypes.c_float * 3)(*(extents or self._ext))
        snp_c = (ctypes.c_float * 3)()

        ref = self._lib.nm_find_nearest_poly(ctypes.c_void_p(self._handle), pos_c, ext_c, snp_c)
        snp_yup = np.array(list(snp_c), dtype=np.float32).reshape(1, 3)
        return int(ref), to_zup(snp_yup)[0]

    def find_path(self, start_zup, end_zup, extents=None):
        start_ref, start_snp = self.find_nearest_poly(start_zup, extents)
        end_ref, end_snp = self.find_nearest_poly(end_zup, extents)

        if start_ref == 0:
            raise ValueError("start point is not on the navmesh")
        if end_ref == 0:
            raise ValueError("end point is not on the navmesh")

        s_yup = to_yup(start_snp.reshape(1, 3))[0]
        e_yup = to_yup(end_snp.reshape(1, 3))[0]
        start_c = vec3f(*s_yup)
        end_c = vec3f(*e_yup)

        polys = (ctypes.c_uint32 * self._max_path)()
        npolys = self._lib.nm_find_path(
            ctypes.c_void_p(self._handle),
            ctypes.c_uint32(start_ref),
            ctypes.c_uint32(end_ref),
            start_c,
            end_c,
            polys,
            self._max_path,
        )
        if npolys <= 0:
            return np.zeros((0, 3), dtype=np.float32)

        path_buf = (ctypes.c_float * (self._max_path * 3))()
        npts = self._lib.nm_find_straight_path(
            ctypes.c_void_p(self._handle), start_c, end_c, polys, npolys, path_buf, self._max_path
        )
        if npts <= 0:
            return np.zeros((0, 3), dtype=np.float32)

        pts_yup = np.frombuffer(path_buf, dtype=np.float32)[: npts * 3].reshape(-1, 3).copy()
        return to_zup(pts_yup).astype(np.float32)
