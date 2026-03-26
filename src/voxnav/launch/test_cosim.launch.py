#!/usr/bin/env python3
"""
Test launch file for co-simulation with fake robot pose.

This launch file starts both the co-simulation node and a fake robot pose
publisher for testing without a real robot.

Usage:
    ros2 launch voxnav test_cosim.launch.py navmesh_file:=/path/to/navmesh.bin
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    navmesh_file_arg = DeclareLaunchArgument(
        'navmesh_file',
        default_value='solo_navmesh.bin',
        description='Path to the navigation mesh .bin file'
    )
    
    motion_type_arg = DeclareLaunchArgument(
        'motion_type',
        default_value='static',
        description='Type of motion: static, linear, or circle'
    )
    
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='-5.0',
        description='Starting X position for fake robot'
    )
    
    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Starting Y position for fake robot'
    )
    
    start_z_arg = DeclareLaunchArgument(
        'start_z',
        default_value='0.0',
        description='Starting Z position for fake robot'
    )
    
    # Fake robot pose publisher node
    fake_robot_node = Node(
        package='voxnav',
        executable='fake_robot_pose',
        name='fake_robot_pose',
        output='screen',
        parameters=[{
            'motion_type': LaunchConfiguration('motion_type'),
            'rate': 30.0,
            'radius': 5.0,
            'speed': 1.0,
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'start_z': LaunchConfiguration('start_z'),
        }]
    )
    
    return LaunchDescription([
        navmesh_file_arg,
        motion_type_arg,
        start_x_arg,
        start_y_arg,
        start_z_arg,
        fake_robot_node,
    ])
