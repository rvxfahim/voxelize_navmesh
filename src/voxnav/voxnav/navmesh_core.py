import ctypes
import numpy as np

from ._coords import to_zup


class NavMesh:
    def __init__(self, bin_path: str, lib):
        self._path = bin_path
        self._lib = lib
        self._handle = self._lib.nm_load(bin_path.encode())
        if not self._handle:
            raise RuntimeError(
                f"Failed to load navmesh from '{bin_path}'. "
                "Check the file is a valid RecastDemo .bin."
            )

    def close(self):
        if self._handle:
            self._lib.nm_free(ctypes.c_void_p(self._handle))
            self._handle = None

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    @property
    def tile_count(self) -> int:
        return self._lib.nm_tile_count(ctypes.c_void_p(self._handle))

    def tile_bounds(self, tile_idx: int):
        bmin_c = (ctypes.c_float * 3)()
        bmax_c = (ctypes.c_float * 3)()
        rc = self._lib.nm_tile_bounds(ctypes.c_void_p(self._handle), tile_idx, bmin_c, bmax_c)
        if rc < 0:
            raise IndexError(f"tile_idx {tile_idx} out of range")
        bmin_yup = np.array(list(bmin_c), dtype=np.float32).reshape(1, 3)
        bmax_yup = np.array(list(bmax_c), dtype=np.float32).reshape(1, 3)
        return to_zup(bmin_yup)[0], to_zup(bmax_yup)[0]

    def get_geometry(self, swap_yz: bool = True):
        all_verts = []
        all_tris = []
        vert_offset = 0

        for ti in range(self.tile_count):
            nf = self._lib.nm_tile_detail_verts(ctypes.c_void_p(self._handle), ti, None, 0)
            if nf <= 0:
                continue
            vbuf = (ctypes.c_float * nf)()
            self._lib.nm_tile_detail_verts(ctypes.c_void_p(self._handle), ti, vbuf, nf)
            verts_yup = np.frombuffer(vbuf, dtype=np.float32).reshape(-1, 3).copy()

            ni = self._lib.nm_tile_detail_tris(ctypes.c_void_p(self._handle), ti, None, 0)
            if ni <= 0:
                continue
            ibuf = (ctypes.c_int * ni)()
            written = self._lib.nm_tile_detail_tris(ctypes.c_void_p(self._handle), ti, ibuf, ni)
            tris = np.frombuffer(ibuf, dtype=np.int32)[:written].reshape(-1, 3).copy()

            all_verts.append(verts_yup)
            all_tris.append(tris + vert_offset)
            vert_offset += len(verts_yup)

        if not all_verts:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int32)

        verts = np.concatenate(all_verts, axis=0)
        tris = np.concatenate(all_tris, axis=0)
        if swap_yz:
            verts = to_zup(verts)
        return verts.astype(np.float32), tris.astype(np.int32)

    def _raw_handle(self):
        return self._handle
