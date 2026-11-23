"""Spawn the moving obstacles into the running Gazebo world and make them pace.

  ros2 launch drone_planning obstacles.launch.py                  # world "building"
  ros2 launch drone_planning obstacles.launch.py world:=building_victims

building.sdf is not changed: walkers are spawned at run time. Ctrl+C stops them
(they stay where they are). To remove one:
  gz service -s /world/building/remove --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "walker_1" type: MODEL'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name, spawn x, y (Gazebo world ENU). Must match config/obstacles.yaml order.
WALKERS = [('walker_1', -5.5, 6.2), ('walker_2', 8.8, 3.0)]  # walker_1 x -5.5 -> 1 (clear of pillar_w and crate_n); walker_2 east of pillar_e, y 3 -> 6.5


def generate_launch_description():
    me = get_package_share_directory('drone_planning')
    sdf = os.path.join(me, 'models', 'walker', 'model.sdf')
    world = LaunchConfiguration('world')

    spawns = [Node(package='ros_gz_sim', executable='create', output='screen',
                   arguments=['-world', world, '-file', sdf, '-name', n,
                              '-x', str(x), '-y', str(y), '-z', '1.0'])
              for n, x, y in WALKERS]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  name='walker_bridge', output='screen',
                  arguments=[f'/model/{n}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
                             for n, _, _ in WALKERS])
    mover = Node(package='drone_planning', executable='obstacle_mover', name='obstacle_mover',
                 output='screen', parameters=[os.path.join(me, 'config', 'obstacles.yaml'),
                                              {'starts_x': [x for _, x, _ in WALKERS],
                                               'starts_y': [y for _, _, y in WALKERS]}])

    # Labels anything in /scan that is not in the (already explored) SLAM map:
    # orange markers on /dynamic_obstacles/markers + "DYNAMIC OBSTACLE #n" log lines.
    tracker = Node(package='drone_planning', executable='dynamic_tracker',
                   name='dynamic_tracker', output='screen',
                   parameters=[{'use_sim_time': True, 'wall_margin': 0.5, 'confirm_hits': 6, 'timeout': 3.0, 'gate': 1.5}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='building'),
        tracker,
        *spawns,
        bridge,
        TimerAction(period=4.0, actions=[mover]),
    ])
