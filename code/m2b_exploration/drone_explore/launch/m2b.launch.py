"""Milestone 2b: GPS-denied drone + Nav2 + cmd_vel -> PX4 bridge.

  ros2 launch drone_explore m2b.launch.py                 # Nav2 mode (plan A)
  ros2 launch drone_explore m2b.launch.py mode:=fallback  # no Nav2 (plan B)

Starts:
  * the M2a stack (bridges, /clock, TFs, rf2o, EV->PX4, slam_toolbox, RViz)
    from slam_drone/m2a.launch.py with waypoints:=false (so the M1/M2a waypoint
    node does NOT fight this bridge for offboard control),
  * cmd_vel_bridge (arms, takes off to 1.5 m, then hovers until told to move),
  * Nav2 (controller, planner, behaviors, bt_navigator), mode:=nav2 only.

Exploration itself is started separately ("press start"):
  ros2 launch drone_explore explore.launch.py [mode:=fallback]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('drone_explore')
    nav2_params = os.path.join(share, 'config', 'nav2_params.yaml')
    bridge_params = os.path.join(share, 'config', 'bridge_params.yaml')

    args = [
        DeclareLaunchArgument('mode', default_value='nav2', description='nav2 or fallback'),
        DeclareLaunchArgument('with_base', default_value='true',
                              description='Also start the M2a stack (false = you run it yourself)'),
        DeclareLaunchArgument('base_pkg', default_value='slam_drone'),
        DeclareLaunchArgument('base_launch', default_value='m2a.launch.py'),
        DeclareLaunchArgument('altitude', default_value='1.5'),
        DeclareLaunchArgument('nav2_delay', default_value='8.0',
                              description='Seconds to wait for SLAM/TF before starting Nav2'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(LaunchConfiguration('base_pkg')), 'launch',
            LaunchConfiguration('base_launch')])),
        launch_arguments={'waypoints': 'false', 'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('with_base')))

    bridge = Node(
        package='drone_explore', executable='cmd_vel_bridge', name='cmd_vel_bridge',
        output='screen',
        parameters=[bridge_params,
                    {'altitude': ParameterValue(LaunchConfiguration('altitude'), value_type=float)}])

    nav2_names = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
    nav2 = GroupAction(
        condition=LaunchConfigurationEquals('mode', 'nav2'),
        actions=[
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_planner', executable='planner_server',
                 name='planner_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server', output='screen', parameters=[nav2_params]),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator', output='screen', parameters=[nav2_params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'use_sim_time': True, 'autostart': True,
                              'node_names': nav2_names, 'bond_timeout': 10.0}]),
        ])

    return LaunchDescription(args + [
        base,
        bridge,
        TimerAction(period=LaunchConfiguration('nav2_delay'), actions=[nav2]),
    ])
