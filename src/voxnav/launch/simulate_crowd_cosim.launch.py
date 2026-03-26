#!/usr/bin/env python3
"""
Launch file for the robot co-simulation node.

This launch file starts the crowd co-simulation that integrates a real robot
with Recast navigation. The robot's position is read from /robotPose and
steering commands are published to /cmd_vel.

Usage:
    ros2 launch voxnav simulate_crowd_cosim.launch.py navmesh_file:=/path/to/navmesh.bin
    
Optional arguments:
    obj_file:=/path/to/mesh.obj
    max_linear_speed:=2.0
    max_angular_speed:=1.5
    update_rate:=30.0
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    navmesh_file_arg = DeclareLaunchArgument(
        'navmesh_file',
        default_value='solo_navmesh.bin',
        description='Path to the navigation mesh .bin file'
    )
    
    obj_file_arg = DeclareLaunchArgument(
        'obj_file',
        default_value='',
        description='Optional path to OBJ file for visualization'
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
    
    # Get configuration values
    navmesh_file = LaunchConfiguration('navmesh_file')
    obj_file = LaunchConfiguration('obj_file')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    update_rate = LaunchConfiguration('update_rate')
    
    # Resolve installed script path
    pkg_prefix = get_package_prefix('voxnav')
    script_path = os.path.join(pkg_prefix, 'bin', 'simulate_crowd_cosim.py')
    
    # Build command with ROS parameters
    cmd = [
        'python3', script_path,
        navmesh_file,
        '--ros-args',
        '-p', ['max_linear_speed:=', max_linear_speed],
        '-p', ['max_angular_speed:=', max_angular_speed],
        '-p', ['update_rate:=', update_rate]
    ]
    
    # For GUI applications, use ExecuteProcess
    simulate_crowd_cosim_process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        shell=False,
    )
    
    return LaunchDescription([
        navmesh_file_arg,
        obj_file_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        update_rate_arg,
        simulate_crowd_cosim_process,
    ])
