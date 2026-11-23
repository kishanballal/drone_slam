"""M2b "start button": begin autonomous exploration.

Run only after the bridge prints  READY at 1.5 m.
  ros2 launch drone_explore explore.launch.py                 # explore_lite + Nav2
  ros2 launch drone_explore explore.launch.py mode:=fallback  # custom frontier node, no Nav2
  ros2 launch drone_explore explore.launch.py mode:=frontier  # custom frontier node -> Nav2 goals
Ctrl+C here stops exploring; the drone hovers. Land with:
  ros2 topic pub --once /drone/land std_msgs/msg/Empty "{}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('drone_explore')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='nav2', description='nav2 (explore_lite), frontier (our explorer + Nav2) or fallback'),
        Node(package='explore_lite', executable='explore', name='explore_node',
             output='screen',
             parameters=[os.path.join(share, 'config', 'explore_params.yaml')],
             condition=LaunchConfigurationEquals('mode', 'nav2')),
        Node(package='drone_explore', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             parameters=[os.path.join(share, 'config', 'frontier_params.yaml')],
             condition=LaunchConfigurationEquals('mode', 'fallback')),
        Node(package='drone_explore', executable='frontier_explorer',
             name='frontier_explorer', output='screen',
             parameters=[os.path.join(share, 'config', 'frontier_params.yaml'),
                         {'output': 'nav2', 'goal_timeout': 60.0}],
             condition=LaunchConfigurationEquals('mode', 'frontier')),
    ])
