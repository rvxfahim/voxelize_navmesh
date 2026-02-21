cmake_minimum_required(VERSION 3.14)
project(recast_cli CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Paths to the pre-built recastnavigation libraries and headers
set(RECAST_DIR ${CMAKE_SOURCE_DIR}/recastnavigation)
set(RECAST_BUILD ${RECAST_DIR}/build)

add_executable(recast_cli recast_cli.cpp)

target_include_directories(recast_cli PRIVATE
    ${RECAST_DIR}/Recast/Include
    ${RECAST_DIR}/Detour/Include
)

target_link_libraries(recast_cli PRIVATE
    ${RECAST_BUILD}/Recast/libRecast.a
    ${RECAST_BUILD}/Detour/libDetour.a
)

# --- navmesh_bridge: Python-callable shared library (ctypes) ----------------
add_library(navmesh_bridge SHARED navmesh_bridge.cpp)

target_include_directories(navmesh_bridge PRIVATE
    ${RECAST_DIR}/Detour/Include
)

target_link_libraries(navmesh_bridge PRIVATE
    ${RECAST_BUILD}/Detour/libDetour.a
)

set_target_properties(navmesh_bridge PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    # Drop the default "lib" prefix so Python can load it as "navmesh_bridge"
    PREFIX ""
    OUTPUT_NAME "navmesh_bridge"
)
