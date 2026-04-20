import ctypes
from dataclasses import dataclass


class nmBuildSettings(ctypes.Structure):
    _fields_ = [
        ("cellSize", ctypes.c_float),
        ("cellHeight", ctypes.c_float),
        ("agentHeight", ctypes.c_float),
        ("agentRadius", ctypes.c_float),
        ("agentMaxClimb", ctypes.c_float),
        ("agentMaxSlope", ctypes.c_float),
        ("regionMinSize", ctypes.c_float),
        ("regionMergeSize", ctypes.c_float),
        ("edgeMaxLen", ctypes.c_float),
        ("edgeMaxError", ctypes.c_float),
        ("vertsPerPoly", ctypes.c_int),
        ("detailSampleDist", ctypes.c_float),
        ("detailSampleMaxError", ctypes.c_float),
        ("partitionType", ctypes.c_int),
        ("filterLowHangingObstacles", ctypes.c_int),
        ("filterLedgeSpans", ctypes.c_int),
        ("filterWalkableLowHeightSpans", ctypes.c_int),
    ]


class dtCrowdAgentParams(ctypes.Structure):
    _fields_ = [
        ("radius", ctypes.c_float),
        ("height", ctypes.c_float),
        ("maxAcceleration", ctypes.c_float),
        ("maxSpeed", ctypes.c_float),
        ("collisionQueryRange", ctypes.c_float),
        ("pathOptimizationRange", ctypes.c_float),
        ("separationWeight", ctypes.c_float),
        ("obstacleWeight", ctypes.c_float),
        ("updateFlags", ctypes.c_ubyte),
        ("obstacleAvoidanceType", ctypes.c_ubyte),
        ("queryFilterType", ctypes.c_ubyte),
        ("userData", ctypes.c_void_p),
    ]


class dtObstacleAvoidanceParams(ctypes.Structure):
    _fields_ = [
        ("velBias", ctypes.c_float),
        ("weightDesVel", ctypes.c_float),
        ("weightCurVel", ctypes.c_float),
        ("weightSide", ctypes.c_float),
        ("weightToi", ctypes.c_float),
        ("horizTime", ctypes.c_float),
        ("gridSize", ctypes.c_ubyte),
        ("adaptiveDivs", ctypes.c_ubyte),
        ("adaptiveRings", ctypes.c_ubyte),
        ("adaptiveDepth", ctypes.c_ubyte),
    ]


@dataclass(frozen=True)
class BoundNavmeshLib:
    lib: ctypes.CDLL
    dt_crowd_agent_params: type
    build_settings_type: type
    dt_obstacle_avoidance_params: type


def bind_navmesh_symbols(lib: ctypes.CDLL) -> BoundNavmeshLib:
    lib.nm_load.restype = ctypes.c_void_p
    lib.nm_load.argtypes = [ctypes.c_char_p]

    lib.nm_free.restype = None
    lib.nm_free.argtypes = [ctypes.c_void_p]

    lib.nm_save.restype = ctypes.c_int
    lib.nm_save.argtypes = [ctypes.c_void_p, ctypes.c_char_p]

    lib.nm_build_solo_from_obj.restype = ctypes.c_void_p
    lib.nm_build_solo_from_obj.argtypes = [
        ctypes.c_char_p,
        ctypes.POINTER(nmBuildSettings),
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
    ]

    lib.nm_tile_count.restype = ctypes.c_int
    lib.nm_tile_count.argtypes = [ctypes.c_void_p]

    lib.nm_tile_bounds.restype = ctypes.c_int
    lib.nm_tile_bounds.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]

    lib.nm_tile_detail_verts.restype = ctypes.c_int
    lib.nm_tile_detail_verts.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float), ctypes.c_int]

    lib.nm_tile_detail_tris.restype = ctypes.c_int
    lib.nm_tile_detail_tris.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int]

    lib.nm_query_create.restype = ctypes.c_void_p
    lib.nm_query_create.argtypes = [ctypes.c_void_p, ctypes.c_int]

    lib.nm_query_free.restype = None
    lib.nm_query_free.argtypes = [ctypes.c_void_p]

    lib.nm_find_nearest_poly.restype = ctypes.c_uint32
    lib.nm_find_nearest_poly.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]

    lib.nm_find_path.restype = ctypes.c_int
    lib.nm_find_path.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint32,
        ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_int,
    ]

    lib.nm_find_straight_path.restype = ctypes.c_int
    lib.nm_find_straight_path.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]

    lib.nm_crowd_create.restype = ctypes.c_void_p
    lib.nm_crowd_create.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]

    lib.nm_crowd_destroy.restype = None
    lib.nm_crowd_destroy.argtypes = [ctypes.c_void_p]

    lib.nm_crowd_add_agent.restype = ctypes.c_int
    lib.nm_crowd_add_agent.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(dtCrowdAgentParams)]

    lib.nm_crowd_remove_agent.restype = None
    lib.nm_crowd_remove_agent.argtypes = [ctypes.c_void_p, ctypes.c_int]

    lib.nm_crowd_request_move_target.restype = ctypes.c_bool
    lib.nm_crowd_request_move_target.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]

    lib.nm_crowd_update.restype = None
    lib.nm_crowd_update.argtypes = [ctypes.c_void_p, ctypes.c_float]

    lib.nm_crowd_get_agent_pos.restype = ctypes.c_bool
    lib.nm_crowd_get_agent_pos.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]

    lib.nm_crowd_request_move_velocity.restype = ctypes.c_bool
    lib.nm_crowd_request_move_velocity.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]

    lib.nm_crowd_teleport_agent.restype = ctypes.c_bool
    lib.nm_crowd_teleport_agent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]

    lib.nm_crowd_force_agent_pos.restype = None
    lib.nm_crowd_force_agent_pos.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]

    lib.nm_crowd_sync_agent_pos.restype = ctypes.c_bool
    lib.nm_crowd_sync_agent_pos.argtypes = [
        ctypes.c_void_p, ctypes.c_int,
        ctypes.POINTER(ctypes.c_float),   # pos
        ctypes.POINTER(ctypes.c_float),   # half_extents (nullable)
    ]

    lib.nm_crowd_set_obstacle_avoidance_params.restype = None
    lib.nm_crowd_set_obstacle_avoidance_params.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(dtObstacleAvoidanceParams)
    ]

    lib.nm_crowd_get_obstacle_avoidance_params.restype = ctypes.c_bool
    lib.nm_crowd_get_obstacle_avoidance_params.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(dtObstacleAvoidanceParams)
    ]

    # --- TileCache build / persistence ---
    lib.nm_build_tiled_with_cache.restype = ctypes.c_void_p
    lib.nm_build_tiled_with_cache.argtypes = [
        ctypes.c_char_p,                  # obj_path
        ctypes.POINTER(nmBuildSettings),  # settings
        ctypes.c_int,                     # input_is_z_up
        ctypes.c_int,                     # tile_size
        ctypes.c_int,                     # max_obstacles
        ctypes.c_char_p,                  # error_out
        ctypes.c_int,                     # error_out_len
    ]

    lib.nm_tc_save.restype = ctypes.c_int
    lib.nm_tc_save.argtypes = [ctypes.c_void_p, ctypes.c_char_p]

    lib.nm_tc_load.restype = ctypes.c_void_p
    lib.nm_tc_load.argtypes = [ctypes.c_char_p]

    lib.nm_tc_free.restype = None
    lib.nm_tc_free.argtypes = [ctypes.c_void_p]

    lib.nm_tc_get_navmesh.restype = ctypes.c_void_p
    lib.nm_tc_get_navmesh.argtypes = [ctypes.c_void_p]

    lib.nm_tc_add_cylinder.restype = ctypes.c_uint32
    lib.nm_tc_add_cylinder.argtypes = [
        ctypes.c_void_p,                      # tc_handle
        ctypes.POINTER(ctypes.c_float),        # pos[3] Y-up base position
        ctypes.c_float,                        # radius
        ctypes.c_float,                        # height
    ]

    lib.nm_tc_remove_obstacle.restype = None
    lib.nm_tc_remove_obstacle.argtypes = [ctypes.c_void_p, ctypes.c_uint32]

    lib.nm_tc_update.restype = None
    lib.nm_tc_update.argtypes = [
        ctypes.c_void_p,               # tc_handle
        ctypes.c_float,                # dt
        ctypes.POINTER(ctypes.c_int),  # upToDate_out (nullable)
    ]

    lib.nm_tc_tile_count.restype = ctypes.c_int
    lib.nm_tc_tile_count.argtypes = [ctypes.c_void_p]

    return BoundNavmeshLib(
        lib=lib, 
        dt_crowd_agent_params=dtCrowdAgentParams, 
        build_settings_type=nmBuildSettings,
        dt_obstacle_avoidance_params=dtObstacleAvoidanceParams
    )
