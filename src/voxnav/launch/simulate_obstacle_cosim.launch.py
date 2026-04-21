#!/usr/bin/env python3
"""
Launch file for the tile-cache obstacle co-simulation node.

Requires a .tcbin file (tiled navmesh + compressed heightfield layers).
Bake one with:
    python3 bake_tcbin.py blender_y.obj blender.tcbin

Usage:
    ros2 launch voxnav simulate_obstacle_cosim.launch.py tcbin_file:=/path/to/scene.tcbin

Optional arguments:
    obj_file:=/path/to/mesh.obj
    max_linear_speed:=2.0
    max_angular_speed:=1.5
    update_rate:=30.0
    robot_radius:=0.3
    dyn_obstacle_source:=cloud
    cluster_voxel_m:=0.3
    cluster_min_voxels:=3
    cylinder_padding:=0.15
    height_padding:=0.20
    max_tc_obstacles:=64
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tcbin_file_arg = DeclareLaunchArgument(
        'tcbin_file',
        default_value='',
        description='Path to .tcbin tiled navmesh file (required)'
    )

    obj_file_arg = DeclareLaunchArgument(
        'obj_file',
        default_value='',
        description='Optional OBJ file for 3-D visualization overlay'
    )

    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='2.0',
        description='Maximum linear velocity (m/s)'
    )

    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='1.5',
        description='Maximum angular velocity (rad/s)'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='Simulation update rate (Hz)'
    )

    kp_angular_arg = DeclareLaunchArgument(
        'kp_angular',
        default_value='2.0',
        description='Proportional gain for angular steering'
    )

    snap_vextent_arg = DeclareLaunchArgument(
        'snap_vextent',
        default_value='-1.0',
        description='Vertical half-extent for navmesh position snap (-1 = auto)'
    )

    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.8',
        description='Physical radius of the robot (m)'
    )

    dyn_obstacle_source_arg = DeclareLaunchArgument(
        'dyn_obstacle_source',
        default_value='cloud',
        description="Dynamic obstacle source: 'cloud' (/foreground_cloud) or 'none'"
    )

    cluster_voxel_m_arg = DeclareLaunchArgument(
        'cluster_voxel_m',
        default_value='0.2',
        description='Voxel grid cell size for point-cloud clustering (m)'
    )

    cluster_min_voxels_arg = DeclareLaunchArgument(
        'cluster_min_voxels',
        default_value='1',
        description='Minimum voxels per cluster (smaller clusters ignored)'
    )

    cylinder_padding_arg = DeclareLaunchArgument(
        'cylinder_padding',
        default_value='0.5',
        description='Extra radius added around each cluster bounding circle (m)'
    )

    height_padding_arg = DeclareLaunchArgument(
        'height_padding',
        default_value='0.55',
        description='Extra height added to each cylinder obstacle (m)'
    )

    max_tc_obstacles_arg = DeclareLaunchArgument(
        'max_tc_obstacles',
        default_value='1024',
        description='Maximum simultaneous tile-cache cylinder obstacles'
    )

    obstacle_decay_s_arg = DeclareLaunchArgument(
        'obstacle_decay_s',
        default_value='0.5',
        description='Seconds to keep an obstacle after it disappears from the point cloud'
    )

    tcbin_file        = LaunchConfiguration('tcbin_file')
    obj_file          = LaunchConfiguration('obj_file')
    max_linear_speed  = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    update_rate       = LaunchConfiguration('update_rate')
    kp_angular        = LaunchConfiguration('kp_angular')
    snap_vextent      = LaunchConfiguration('snap_vextent')
    robot_radius      = LaunchConfiguration('robot_radius')
    dyn_obstacle_source = LaunchConfiguration('dyn_obstacle_source')
    cluster_voxel_m   = LaunchConfiguration('cluster_voxel_m')
    cluster_min_voxels = LaunchConfiguration('cluster_min_voxels')
    cylinder_padding  = LaunchConfiguration('cylinder_padding')
    height_padding    = LaunchConfiguration('height_padding')
    max_tc_obstacles  = LaunchConfiguration('max_tc_obstacles')
    obstacle_decay_s  = LaunchConfiguration('obstacle_decay_s')

    pkg_prefix = get_package_prefix('voxnav')
    script_path = os.path.join(pkg_prefix, 'lib', 'voxnav', 'simulate_obstacle_cosim.py')

    cmd = [
        'python3', script_path,
        tcbin_file,
        '--obj', obj_file,
        '--ros-args',
        '-p', ['max_linear_speed:=', max_linear_speed],
        '-p', ['max_angular_speed:=', max_angular_speed],
        '-p', ['update_rate:=', update_rate],
        '-p', ['kp_angular:=', kp_angular],
        '-p', ['snap_vextent:=', snap_vextent],
        '-p', ['robot_radius:=', robot_radius],
        '-p', ['dyn_obstacle_source:=', dyn_obstacle_source],
        '-p', ['cluster_voxel_m:=', cluster_voxel_m],
        '-p', ['cluster_min_voxels:=', cluster_min_voxels],
        '-p', ['cylinder_padding:=', cylinder_padding],
        '-p', ['height_padding:=', height_padding],
        '-p', ['max_tc_obstacles:=', max_tc_obstacles],
        '-p', ['obstacle_decay_s:=', obstacle_decay_s],
    ]

    simulate_process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        shell=False,
    )

    return LaunchDescription([
        tcbin_file_arg,
        obj_file_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        update_rate_arg,
        kp_angular_arg,
        snap_vextent_arg,
        robot_radius_arg,
        dyn_obstacle_source_arg,
        cluster_voxel_m_arg,
        cluster_min_voxels_arg,
        cylinder_padding_arg,
        height_padding_arg,
        max_tc_obstacles_arg,
        obstacle_decay_s_arg,
        simulate_process,
    ])
