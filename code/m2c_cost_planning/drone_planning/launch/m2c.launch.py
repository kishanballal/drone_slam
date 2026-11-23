"""Milestone 2c: cost-aware planning + real-time replanning (professor's list).

  ros2 launch drone_planning m2c.launch.py

Starts:
  * M2b in fallback mode = M2a GPS-denied stack (rf2o -> EKF2, slam_toolbox, RViz)
    + cmd_vel_bridge (arms, takes off to 1.5 m, hovers). M2b itself is unchanged.
  * cost_planner (our planner, J = distance + risk + battery + complexity)
  * Nav2 (controller, planner_server for the global costmap, behaviors,
    bt_navigator using behavior_trees/nav_to_pose_cost.xml -> cost_planner at 2 Hz)

Then, in other tabs:
  ros2 launch drone_planning obstacles.launch.py     # moving obstacles
  ros2 launch drone_explore explore.launch.py        # autonomous exploration
  (or click "2D Goal Pose" in RViz)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    explore = get_package_share_directory('drone_explore')
    me = get_package_share_directory('drone_planning')
    nav2_params = os.path.join(explore, 'config', 'nav2_params.yaml')
    overrides = os.path.join(me, 'config', 'nav2_overrides.yaml')
    bt_xml = os.path.join(me, 'behavior_trees', 'nav_to_pose_cost.xml')
    params = [nav2_params, overrides]

    m2b = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(explore, 'launch', 'm2b.launch.py')),
        launch_arguments={'mode': 'fallback'}.items())

    planner = Node(package='drone_planning', executable='cost_planner', name='cost_planner',
                   output='screen',
                   parameters=[os.path.join(me, 'config', 'cost_planner.yaml')])

    names = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
    nav2 = GroupAction([
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=params),
        # planner_server stays: it owns the global costmap (+ its clear services).
        # Its own navfn planner is no longer used by the tree.
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=params),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=params),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=params + [{'default_nav_to_pose_bt_xml': bt_xml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': names, 'bond_timeout': 10.0}]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('nav2_delay', default_value='8.0'),
        m2b,
        planner,
        TimerAction(period=LaunchConfiguration('nav2_delay'), actions=[nav2]),
    ])
