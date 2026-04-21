"""
voxelizer.launch.py
────────────────────
Launches the voxelizer_node with the bundled param file.

Optional arguments
  rviz  (default true)  – open RViz2 with the bundled config
  leaf_size             – override the leaf size at launch time

Examples
  ros2 launch pointcloud_voxelizer voxelizer.launch.py
  ros2 launch pointcloud_voxelizer voxelizer.launch.py rviz:=false
  ros2 launch pointcloud_voxelizer voxelizer.launch.py leaf_size:=0.2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('pointcloud_voxelizer')
    param_file = os.path.join(pkg_share, 'param', 'voxelizer.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'voxelizer.rviz')

    # ---- launch arguments --------------------------------------------------
    arg_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with the bundled voxelizer config')

    arg_leaf = DeclareLaunchArgument(
        'leaf_size',
        default_value='',
        description='Override leaf_size parameter (metres). Leave empty to use param file value.')

    # ---- voxelizer node ----------------------------------------------------
    def make_voxelizer_node(context, *args, **kwargs):
        leaf_override = LaunchConfiguration('leaf_size').perform(context).strip()
        extra_params = []
        if leaf_override:
            extra_params.append({'leaf_size': float(leaf_override)})

        return [Node(
            package='pointcloud_voxelizer',
            executable='voxelizer_node',
            name='voxelizer_node',
            output='screen',
            parameters=[param_file] + extra_params,
            # Remap here if the raw topic name differs in your setup:
            # remappings=[('/hesai/pandar', '/your_lidar_topic')],
        )]

    # ---- RViz2 (optional) --------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        arg_rviz,
        arg_leaf,
        OpaqueFunction(function=make_voxelizer_node),
        rviz_node,
    ])
