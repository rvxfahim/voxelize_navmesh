cmake_minimum_required(VERSION 3.14)
project(voxnav)

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find ament_cmake and ament_python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(RECAST_DIR ${CMAKE_SOURCE_DIR}/../recastnavigation)

set(_recast_old_pic ${CMAKE_POSITION_INDEPENDENT_CODE})
set(_recast_old_demo ${RECASTNAVIGATION_DEMO})
set(_recast_old_tests ${RECASTNAVIGATION_TESTS})
set(_recast_old_examples ${RECASTNAVIGATION_EXAMPLES})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "Disable Recast demo in voxnav build" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "Disable Recast tests in voxnav build" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "Disable Recast examples in voxnav build" FORCE)
add_subdirectory(${RECAST_DIR} ${CMAKE_BINARY_DIR}/recastnavigation)
set(CMAKE_POSITION_INDEPENDENT_CODE ${_recast_old_pic})
if(DEFINED _recast_old_demo)
  set(RECASTNAVIGATION_DEMO ${_recast_old_demo})
endif()
if(DEFINED _recast_old_tests)
  set(RECASTNAVIGATION_TESTS ${_recast_old_tests})
endif()
if(DEFINED _recast_old_examples)
  set(RECASTNAVIGATION_EXAMPLES ${_recast_old_examples})
endif()

# Build the navmesh_bridge shared library
add_library(navmesh_bridge SHARED
  voxnav/navmesh_bridge.cpp
)

target_include_directories(navmesh_bridge PRIVATE
  ${RECAST_DIR}/Recast/Include
  ${RECAST_DIR}/Detour/Include
  ${RECAST_DIR}/DetourCrowd/Include
)

target_link_libraries(navmesh_bridge PUBLIC
  Recast
  DetourCrowd
  Detour
  pthread
  m
)

set_target_properties(navmesh_bridge PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  PREFIX ""
  OUTPUT_NAME "navmesh_bridge"
)

# Install targets
install(
  TARGETS navmesh_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(voxnav)

# Install the simulate_crowd.py script as an executable
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/voxnav/simulate_crowd.py
  ${CMAKE_CURRENT_SOURCE_DIR}/voxnav/navmesh_baker.py
  DESTINATION bin
)

# Make the script executable
set(BIN_INSTALL_DIR "bin")

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
