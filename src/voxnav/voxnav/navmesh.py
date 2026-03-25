"""
Public navmesh API and compatibility facade.

This module now composes smaller internal components:
- shared-library loading
- ctypes ABI bindings
- wrapper classes (NavMesh/NavMeshQuery/Crowd)
"""

from dataclasses import dataclass

from ._bridge_loader import load_bridge_lib
from ._ctypes_bindings import bind_navmesh_symbols
from .crowd import Crowd as _Crowd
from .navmesh_core import NavMesh as _NavMesh
from .navmesh_query import NavMeshQuery as _NavMeshQuery

_BOUND = bind_navmesh_symbols(load_bridge_lib())
_LIB = _BOUND.lib


class NavMesh(_NavMesh):
    def __init__(self, bin_path: str):
        super().__init__(bin_path=bin_path, lib=_LIB)


class NavMeshQuery(_NavMeshQuery):
    def __init__(self, navmesh: NavMesh, max_nodes: int = 2048, extents=(2.0, 4.0, 2.0), max_path: int = 1024):
        super().__init__(navmesh=navmesh, lib=_LIB, max_nodes=max_nodes, extents=extents, max_path=max_path)


class Crowd(_Crowd):
    def __init__(self, navmesh: NavMesh, max_agents: int = 100, max_agent_radius: float = 2.0):
        super().__init__(
            navmesh=navmesh,
            lib=_LIB,
            crowd_params_type=_BOUND.dt_crowd_agent_params,
            max_agents=max_agents,
            max_agent_radius=max_agent_radius,
        )


@dataclass(frozen=True)
class NavRuntime:
    lib: object
    crowd_params_type: type

    def navmesh(self, bin_path: str) -> _NavMesh:
        return _NavMesh(bin_path=bin_path, lib=self.lib)

    def navquery(self, navmesh: _NavMesh, max_nodes: int = 2048, extents=(2.0, 4.0, 2.0), max_path: int = 1024) -> _NavMeshQuery:
        return _NavMeshQuery(navmesh=navmesh, lib=self.lib, max_nodes=max_nodes, extents=extents, max_path=max_path)

    def crowd(self, navmesh: _NavMesh, max_agents: int = 100, max_agent_radius: float = 2.0) -> _Crowd:
        return _Crowd(
            navmesh=navmesh,
            lib=self.lib,
            crowd_params_type=self.crowd_params_type,
            max_agents=max_agents,
            max_agent_radius=max_agent_radius,
        )


def create_runtime() -> NavRuntime:
    return NavRuntime(lib=_LIB, crowd_params_type=_BOUND.dt_crowd_agent_params)


__all__ = ["NavMesh", "NavMeshQuery", "Crowd", "NavRuntime", "create_runtime"]
