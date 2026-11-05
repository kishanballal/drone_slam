"""Milestone 1: the drone maps the building.

Start PX4 + Gazebo first (Tab 1), then:
    ros2 launch slam_drone m1.launch.py              # mapping only (fly with commander)
    ros2 launch slam_drone m1.launch.py fly:=true    # waypoint node flies the corridor loop

Starts: MicroXRCEAgent, Gazebo->ROS bridge (/clock, /scan, /lidar_3d/points),
static sensor TFs, PX4 odometry -> TF node, slam_toolbox (online_async), RViz,
and (fly:=true) the waypoint offboard node. Every ROS node uses sim time.
"""
import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM = {'use_sim_time': True}


def gz_topics():
    try:
        out = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=10).stdout
        return [t.strip() for t in out.splitlines() if t.strip()]
    except Exception:
        return []


def gz_frame_id(topic):
    """Read one message from a Gazebo topic and return its header frame_id."""
    try:
        out = subprocess.run(['gz', 'topic', '-e', '-t', topic, '-n', '1'],
                             capture_output=True, text=True, timeout=10).stdout
    except Exception:
        return ''
    m = re.search(r'key:\s*"frame_id"\s*value:\s*"([^"]+)"', out)
    if m:
        return m.group(1)
    m = re.search(r'^\s*frame:\s*"([^"]+)"', out, re.M)
    return m.group(1) if m else ''


def setup(context):
    actions = []
    topics = gz_topics()
    if not topics:
        return [LogInfo(msg='ERROR: no Gazebo topics found. Start PX4 + Gazebo in Tab 1 first, then relaunch.')]

    scan_gz = LaunchConfiguration('gz_scan_topic').perform(context)
    if not scan_gz:
        scans = [t for t in topics if t.endswith('/scan')]
        scan_gz = scans[0] if scans else ''
    if not scan_gz:
        return [LogInfo(msg='ERROR: no Gazebo topic ending in /scan. Is the x500_lidar_3d drone spawned?')]

    scan_frame = LaunchConfiguration('scan_frame').perform(context) or gz_frame_id(scan_gz)
    cloud_frame = LaunchConfiguration('cloud_frame').perform(context) or gz_frame_id('/lidar_3d/points')
    if not scan_frame or not cloud_frame:
        return [LogInfo(msg=f'ERROR: could not read sensor frame ids (scan="{scan_frame}", '
                            f'cloud="{cloud_frame}"). Pass them: scan_frame:=... cloud_frame:=...')]

    actions.append(LogInfo(msg=f'2D scan: {scan_gz}  frame: {scan_frame}'))
    actions.append(LogInfo(msg=f'3D cloud frame: {cloud_frame}'))

    # Gazebo -> ROS 2 bridge (clock + both LiDARs). Bridge itself runs on wall time.
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/lidar_3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'{scan_gz}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[(scan_gz, '/scan')],
    ))

    # Static sensor transforms (same offsets as M0)
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_lidar_3d',
        arguments=['--x', '0', '--y', '0', '--z', '0.32',
                   '--frame-id', 'base_link', '--child-frame-id', cloud_frame],
        parameters=[SIM]))
    actions.append(Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_lidar_2d',
        arguments=['--x', '0.12', '--y', '0', '--z', '0.26',
                   '--frame-id', 'base_link', '--child-frame-id', scan_frame],
        parameters=[SIM]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('slam_drone')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_m1.yaml')
    rviz_cfg = os.path.join(pkg, 'config', 'm1.rviz')
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'),
                               'launch', 'online_async_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('fly', default_value='false',
                              description='Start the waypoint node (flies the loop on its own)'),
        DeclareLaunchArgument('px4_odom_tf', default_value='true',
                              description='PX4 odometry -> odom->base_link TF (false in M2a, rf2o owns it)'),
        DeclareLaunchArgument('agent', default_value='true', description='Start MicroXRCEAgent'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('map3d', default_value='false',
                              description='Also build the 3D height map (needs slam_drone_m3 built)'),
        DeclareLaunchArgument('odom_topic', default_value='/fmu/out/vehicle_odometry',
                              description='PX4 odometry topic (check: ros2 topic list | grep odom)'),
        DeclareLaunchArgument('gz_scan_topic', default_value='', description='Empty = auto-detect'),
        DeclareLaunchArgument('scan_frame', default_value='', description='Empty = auto-detect'),
        DeclareLaunchArgument('cloud_frame', default_value='', description='Empty = auto-detect'),

        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='log',
                       condition=IfCondition(LaunchConfiguration('agent'))),

        OpaqueFunction(function=setup),

        Node(package='slam_drone', executable='odom_tf', name='odom_tf', output='screen',
             parameters=[SIM, {'odom_topic': LaunchConfiguration('odom_topic')}],
             condition=IfCondition(LaunchConfiguration('px4_odom_tf'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': slam_params}.items()),

        Node(package='slam_drone_m3', executable='height_map_node', name='height_map_3d',
             output='screen', parameters=[SIM],
             condition=IfCondition(LaunchConfiguration('map3d'))),

        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg],
             parameters=[SIM], condition=IfCondition(LaunchConfiguration('rviz'))),

        # Give SLAM and the bridges ~8 s to come up before the drone takes off.
        TimerAction(period=8.0, actions=[
            Node(package='slam_drone', executable='waypoint_flight', name='waypoint_flight',
                 output='screen', parameters=[SIM],
                 condition=IfCondition(LaunchConfiguration('fly')))]),
    ])
