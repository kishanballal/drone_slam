"""M4: search-route planner + executor. Start it together with exploration, then
   ros2 topic pub --once /search_route/start std_msgs/msg/Empty "{}"
when exploration is finished."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('drone_search'), 'config', 'search_route.yaml')
    return LaunchDescription([
        Node(package='drone_search', executable='search_route', name='search_route',
             output='screen', parameters=[cfg]),
    ])
