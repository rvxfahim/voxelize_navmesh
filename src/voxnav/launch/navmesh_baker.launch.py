#!/usr/bin/env python3
"""
Launch file for the PyQt/Open3D navmesh baker GUI.

Usage:
    ros2 launch voxnav navmesh_baker.launch.py
"""

import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_prefix = get_package_prefix("voxnav")
    script_path = os.path.join(pkg_prefix, "bin", "navmesh_baker.py")

    navmesh_baker_process = ExecuteProcess(
        cmd=["python3", script_path],
        output="screen",
        shell=False,
    )

    return LaunchDescription([navmesh_baker_process])
