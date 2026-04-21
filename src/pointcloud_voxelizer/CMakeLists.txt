cmake_minimum_required(VERSION 3.5)
project(pointcloud_voxelizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")

# ---- find packages -------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- build target --------------------------------------------------------
add_executable(voxelizer_node src/voxelizer_node.cpp)

ament_target_dependencies(voxelizer_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

target_include_directories(voxelizer_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(voxelizer_node
  ${PCL_LIBRARIES}
)

target_compile_definitions(voxelizer_node PRIVATE
  ${PCL_DEFINITIONS}
)

# ---- install -------------------------------------------------------------
install(TARGETS voxelizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch param rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
