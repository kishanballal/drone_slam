"""Milestone 3 add-on: 3D height map + victim detector + RViz.

Start your M2 demo first (sim + M2 launch). This file only ADDS nodes, so M0-M2 stay untouched.
  ros2 launch slam_drone_m3 m3.launch.py            # with RViz
  ros2 launch slam_drone_m3 m3.launch.py rviz:=false
  ros2 launch slam_drone_m3 m3.launch.py victims:=false   # 3D map only (M3 step 1 demo)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('slam_drone_m3')
    params = os.path.join(share, 'config', 'm3_params.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'm3.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('victims', default_value='true'),
        Node(package='slam_drone_m3', executable='height_map_node', name='height_map_3d',
             parameters=[params], output='screen'),
        Node(package='slam_drone_m3', executable='victim_detector_node', name='victim_detector',
             parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('victims'))),
        Node(package='rviz2', executable='rviz2', name='rviz_m3', arguments=['-d', rviz_cfg],
             parameters=[{'use_sim_time': True}], output='log',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
