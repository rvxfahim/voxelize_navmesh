import ctypes
import numpy as np

from ._coords import to_yup, to_zup


class Crowd:
    def __init__(self, navmesh, lib, crowd_params_type, max_agents: int = 100, max_agent_radius: float = 2.0):
        self._nm = navmesh
        self._lib = lib
        self._crowd_params_type = crowd_params_type
        self._handle = self._lib.nm_crowd_create(ctypes.c_void_p(navmesh._raw_handle()), max_agents, max_agent_radius)
        if not self._handle:
            raise RuntimeError("Failed to create Crowd")

    def close(self):
        if self._handle:
            self._lib.nm_crowd_destroy(ctypes.c_void_p(self._handle))
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
        return (ctypes.c_float * 3)(float(yup[0]), float(yup[1]), float(yup[2]))

    def add_agent(self, pos_zup, radius=0.3, height=2.0, maxAcceleration=8.0, maxSpeed=3.5, collisionQueryRange=12.0):
        pos_c = self._to_c3(pos_zup)
        params = self._crowd_params_type()
        params.radius = float(radius)
        params.height = float(height)
        params.maxAcceleration = float(maxAcceleration)
        params.maxSpeed = float(maxSpeed)
        params.collisionQueryRange = float(collisionQueryRange)
        params.pathOptimizationRange = float(radius * 30.0)
        params.separationWeight = 2.0
        params.updateFlags = 1 | 2 | 4 | 8
        params.obstacleAvoidanceType = 0
        params.queryFilterType = 0
        params.userData = None

        idx = self._lib.nm_crowd_add_agent(ctypes.c_void_p(self._handle), pos_c, ctypes.byref(params))
        if idx < 0:
            raise RuntimeError("Failed to add agent to crowd (pool full?)")
        return idx

    def remove_agent(self, idx: int):
        self._lib.nm_crowd_remove_agent(ctypes.c_void_p(self._handle), idx)

    def request_move_target(self, idx: int, pos_zup, navquery):
        poly_ref, snp = navquery.find_nearest_poly(pos_zup)
        if poly_ref == 0:
            return False
        pos_c = self._to_c3(snp)
        return self._lib.nm_crowd_request_move_target(ctypes.c_void_p(self._handle), idx, poly_ref, pos_c)

    def request_move_velocity(self, idx: int, vel_zup):
        vel_c = self._to_c3(vel_zup)
        return self._lib.nm_crowd_request_move_velocity(ctypes.c_void_p(self._handle), idx, vel_c)

    def force_agent_pos(self, idx: int, pos_zup):
        pos_c = self._to_c3(pos_zup)
        self._lib.nm_crowd_force_agent_pos(ctypes.c_void_p(self._handle), idx, pos_c)

    def sync_agent_pos(self, idx: int, pos_zup, snap_vextent: float | None = None) -> bool:
        """Snap agent to nearest navmesh poly and update corridor without resetting the move target.

        snap_vextent: vertical half-extent (Y-up metres) for the nearest-poly search.
        Use a value smaller than half the floor separation to avoid cross-floor snapping
        in multi-floor environments (e.g. 1.0 for ~3 m floor pitch). None = crowd default.
        """
        pos_c = self._to_c3(pos_zup)
        if snap_vextent is not None:
            ext_c = (ctypes.c_float * 3)(2.0, float(snap_vextent), 2.0)
            ext_ptr = ctypes.cast(ext_c, ctypes.POINTER(ctypes.c_float))
        else:
            ext_ptr = ctypes.cast(None, ctypes.POINTER(ctypes.c_float))
        return bool(self._lib.nm_crowd_sync_agent_pos(ctypes.c_void_p(self._handle), idx, pos_c, ext_ptr))

    def teleport_agent(self, idx: int, pos_zup):
        pos_c = self._to_c3(pos_zup)
        return bool(self._lib.nm_crowd_teleport_agent(ctypes.c_void_p(self._handle), idx, pos_c))

    def get_agent_pos(self, idx: int):
        pos_c = (ctypes.c_float * 3)()
        vel_c = (ctypes.c_float * 3)()
        if self._lib.nm_crowd_get_agent_pos(ctypes.c_void_p(self._handle), idx, pos_c, vel_c):
            pos_yup = np.array(list(pos_c), dtype=np.float32).reshape(1, 3)
            vel_yup = np.array(list(vel_c), dtype=np.float32).reshape(1, 3)
            return to_zup(pos_yup)[0], to_zup(vel_yup)[0]
        return None, None

    def update(self, dt: float):
        self._lib.nm_crowd_update(ctypes.c_void_p(self._handle), ctypes.c_float(dt))
