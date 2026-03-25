#!/usr/bin/env python3
"""
Launch file for the crowd simulation node.

Usage:
    ros2 launch voxnav simulate_crowd.launch.py navmesh_file:=/path/to/navmesh.bin
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare the navmesh file path argument
    navmesh_file_arg = DeclareLaunchArgument(
        'navmesh_file',
        default_value='solo_navmesh.bin',
        description='Path to the navigation mesh .bin file'
    )

    # Get the navmesh file path
    navmesh_file = LaunchConfiguration('navmesh_file')

    # Resolve installed script and package prefix.
    pkg_prefix = get_package_prefix('voxnav')
    script_path = os.path.join(pkg_prefix, 'bin', 'simulate_crowd.py')

    # For GUI applications (Tkinter/Open3D), use ExecuteProcess instead of Node
    # This runs the Python script directly
    simulate_crowd_process = ExecuteProcess(
        cmd=['python3', script_path, navmesh_file],
        output='screen',
        shell=False,
    )

    return LaunchDescription([
        navmesh_file_arg,
        simulate_crowd_process,
    ])
