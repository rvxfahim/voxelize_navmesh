import ctypes
import os


def _candidate_library_paths():
    here = os.path.dirname(os.path.abspath(__file__))
    paths = []

    try:
        from ament_index_python.packages import get_package_prefix

        try:
            pkg_prefix = get_package_prefix("voxnav")
            paths.append(os.path.join(pkg_prefix, "lib", "navmesh_bridge.so"))
            paths.append(os.path.join(pkg_prefix, "lib", "voxnav", "navmesh_bridge.so"))
        except Exception:
            pass
    except ImportError:
        pass

    pkg_dir = os.path.dirname(here)  # src/voxnav/
    repo_dir = os.path.dirname(os.path.dirname(pkg_dir))  # repo root
    paths.extend(
        [
            os.path.join(here, "build", "navmesh_bridge.so"),
            os.path.join(here, "navmesh_bridge.so"),
            os.path.join(pkg_dir, "build", "navmesh_bridge.so"),
            os.path.join(pkg_dir, "build", "voxnav", "navmesh_bridge.so"),
            os.path.join(repo_dir, "build", "voxnav", "navmesh_bridge.so"),
            os.path.join(repo_dir, "install", "voxnav", "lib", "navmesh_bridge.so"),
            os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(here))), "lib", "navmesh_bridge.so"),
        ]
    )
    return paths


def load_bridge_lib():
    for path in _candidate_library_paths():
        if os.path.exists(path):
            return ctypes.CDLL(path)
    raise FileNotFoundError(
        "navmesh_bridge.so not found. Build it with:\n"
        "  cmake --build build --target navmesh_bridge"
    )
