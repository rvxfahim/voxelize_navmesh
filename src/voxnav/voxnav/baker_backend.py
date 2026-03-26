import ctypes
from dataclasses import dataclass

import numpy as np

from ._bridge_loader import load_bridge_lib
from ._ctypes_bindings import bind_navmesh_symbols
from ._coords import to_zup
from .recast_config import RecastBuildConfig


_BOUND = bind_navmesh_symbols(load_bridge_lib())
_LIB = _BOUND.lib


@dataclass
class BakedNavmesh:
    handle: int
    lib: ctypes.CDLL

    def close(self) -> None:
        if self.handle:
            self.lib.nm_free(ctypes.c_void_p(self.handle))
            self.handle = 0

    def save(self, path: str) -> None:
        rc = self.lib.nm_save(ctypes.c_void_p(self.handle), path.encode("utf-8"))
        if rc != 0:
            raise RuntimeError(f"Failed to save navmesh to '{path}'")

    def get_geometry(self, swap_yz: bool = False):
        tile_count = int(self.lib.nm_tile_count(ctypes.c_void_p(self.handle)))
        all_verts = []
        all_tris = []
        vert_offset = 0
        for ti in range(tile_count):
            nf = int(self.lib.nm_tile_detail_verts(ctypes.c_void_p(self.handle), ti, None, 0))
            if nf <= 0:
                continue
            vbuf = (ctypes.c_float * nf)()
            written_v = int(self.lib.nm_tile_detail_verts(ctypes.c_void_p(self.handle), ti, vbuf, nf))
            if written_v <= 0:
                continue
            verts_yup = np.frombuffer(vbuf, dtype=np.float32)[:written_v].reshape(-1, 3).copy()

            ni = int(self.lib.nm_tile_detail_tris(ctypes.c_void_p(self.handle), ti, None, 0))
            if ni <= 0:
                continue
            ibuf = (ctypes.c_int * ni)()
            written_i = int(self.lib.nm_tile_detail_tris(ctypes.c_void_p(self.handle), ti, ibuf, ni))
            if written_i <= 0:
                continue
            tris = np.frombuffer(ibuf, dtype=np.int32)[:written_i].reshape(-1, 3).copy()

            all_verts.append(verts_yup)
            all_tris.append(tris + vert_offset)
            vert_offset += len(verts_yup)

        if not all_verts:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.int32)
        verts = np.concatenate(all_verts, axis=0)
        tris = np.concatenate(all_tris, axis=0)
        if swap_yz:
            verts = to_zup(verts)
        return verts.astype(np.float32), tris.astype(np.int32)


def _to_c_settings(cfg: RecastBuildConfig):
    cfg.validate()
    s = _BOUND.build_settings_type()
    s.cellSize = float(cfg.cell_size)
    s.cellHeight = float(cfg.cell_height)
    s.agentHeight = float(cfg.agent_height)
    s.agentRadius = float(cfg.agent_radius)
    s.agentMaxClimb = float(cfg.agent_max_climb)
    s.agentMaxSlope = float(cfg.agent_max_slope)
    s.regionMinSize = float(cfg.region_min_size)
    s.regionMergeSize = float(cfg.region_merge_size)
    s.edgeMaxLen = float(cfg.edge_max_len)
    s.edgeMaxError = float(cfg.edge_max_error)
    s.vertsPerPoly = int(cfg.verts_per_poly)
    s.detailSampleDist = float(cfg.detail_sample_dist)
    s.detailSampleMaxError = float(cfg.detail_sample_max_error)
    s.partitionType = int(cfg.partition_type)
    s.filterLowHangingObstacles = 1 if cfg.filter_low_hanging_obstacles else 0
    s.filterLedgeSpans = 1 if cfg.filter_ledge_spans else 0
    s.filterWalkableLowHeightSpans = 1 if cfg.filter_walkable_low_height_spans else 0
    return s


def build_navmesh_from_obj(obj_path: str, cfg: RecastBuildConfig, input_is_z_up: bool = True) -> BakedNavmesh:
    c_settings = _to_c_settings(cfg)
    err = ctypes.create_string_buffer(1024)
    handle = _LIB.nm_build_solo_from_obj(
        obj_path.encode("utf-8"),
        ctypes.byref(c_settings),
        1 if input_is_z_up else 0,
        err,
        len(err),
    )
    if not handle:
        msg = err.value.decode("utf-8", errors="replace").strip() or "Unknown navmesh build error."
        raise RuntimeError(msg)
    return BakedNavmesh(handle=int(handle), lib=_LIB)
