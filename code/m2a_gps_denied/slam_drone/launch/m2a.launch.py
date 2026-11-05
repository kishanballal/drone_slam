"""Milestone 2a: M1 mapping flight with GPS off.

Starts everything from m1.launch.py (MicroXRCEAgent, bridges, static TFs,
slam_toolbox, RViz) EXCEPT the PX4 odom_tf node and the
waypoint node, then adds rf2o (owns odom -> base_link and /odom) and the
laser_odom_to_px4 bridge (feeds EKF2). Start the waypoint flight yourself once
EKF2 reports it is fusing external vision:
    ros2 run slam_drone waypoint_flight
or pass waypoints:=true to start it from here (20 s after launch; it waits for
a valid local position before arming anyway). M2b includes this file with
waypoints:=false (the default) so only its cmd_vel bridge commands PX4.

PX4 must be started with the GPS-denied airframe:
    PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('slam_drone')

    m1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'm1.launch.py')),
        launch_arguments={'px4_odom_tf': 'false', 'fly': 'false'}.items(),
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'rf2o.yaml')],
    )

    to_px4 = Node(
        package='slam_drone',
        executable='laser_odom_to_px4',
        name='laser_odom_to_px4',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    waypoints = Node(
        package='slam_drone',
        executable='waypoint_flight',
        name='waypoint_flight',
        output='screen',
        condition=IfCondition(LaunchConfiguration('waypoints')),
    )

    # rf2o reads the base_link -> laser TF once, on its first scan. Give the
    # static TFs from m1.launch.py a few seconds to exist first.
    return LaunchDescription([
        DeclareLaunchArgument('waypoints', default_value='false',
                              description='true = also start the M1 waypoint flight (20 s delay)'),
        m1,
        TimerAction(period=5.0, actions=[rf2o, to_px4]),
        TimerAction(period=20.0, actions=[waypoints]),
    ])
