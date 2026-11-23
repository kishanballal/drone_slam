"""M3v: visual SLAM (RTAB-Map RGB-D) next to the running M2a/M2b/M2c stack.

PX4 must run the camera airframe:
  PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d_ev_cam

  ros2 launch drone_vslam vslam.launch.py                     # camera mapping + visual loop closure,
                                                              # motion prior from rf2o odom (robust)
  ros2 launch drone_vslam vslam.launch.py visual_odom:=true   # pure camera odometry too (experiment)

Frames: RTAB-Map works in its own frames (vmap, and vodom when visual_odom:=true)
and never publishes TF into map/odom, so slam_toolbox, rf2o, EKF2 and Nav2 are
untouched. A static identity map -> vmap lets RViz show both maps together
(both start at the take-off point).
RViz: PointCloud2 /rtabmap/cloud_map, Path /rtabmap/mapPath, Odometry /vo/odom.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAM_REMAPS = [('rgb/image', '/camera/image'), ('depth/image', '/camera/depth_image'),
              ('rgb/camera_info', '/camera/camera_info')]

COMMON = {
    'use_sim_time': True,
    'frame_id': 'base_link',
    'subscribe_depth': True,
    'approx_sync': True,
    'approx_sync_max_interval': 0.05,
    'queue_size': 10,
    'qos': 1,                      # sensor topics are reliable from ros_gz_bridge
    'wait_for_transform': 0.3,
}

SLAM = {
    **COMMON,
    'map_frame_id': 'vmap',
    'publish_tf': False,
    'Rtabmap/DetectionRate': '1',          # 1 Hz map updates: low CPU / memory
    'Mem/ImagePreDecimation': '1',
    'Mem/ImagePostDecimation': '2',        # keep half-size images in the database
    'Mem/STMSize': '15',
    'RGBD/LinearUpdate': '0.15',
    'RGBD/AngularUpdate': '0.15',
    'Grid/RangeMax': '6.0',
    'Grid/CellSize': '0.1',
    'Vis/MinInliers': '12',
    'Kp/MaxFeatures': '300',
}


def generate_launch_description():
    vo = LaunchConfiguration('visual_odom')
    return LaunchDescription([
        DeclareLaunchArgument('visual_odom', default_value='false'),

        Node(package='drone_vslam', executable='camera_guard', name='camera_guard',
             output='screen', parameters=[{'use_sim_time': True,
                                           'max_minutes': 25.0}]),  # exploration + camera tour

        # camera mount (same pose as in the model) + ROS optical frame
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_cam',
             arguments=['--x', '0.15', '--z', '0.05', '--frame-id', 'base_link',
                        '--child-frame-id', 'camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_cam_opt',
             arguments=['--roll', '-1.5708', '--yaw', '-1.5708', '--frame-id', 'camera_link',
                        '--child-frame-id', 'camera_optical']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_vmap',
             arguments=['--frame-id', 'map', '--child-frame-id', 'vmap']),

        # pure visual odometry (experiment); publishes /vo/odom only, no TF
        Node(package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
             output='screen', condition=IfCondition(vo),
             parameters=[{**COMMON, 'odom_frame_id': 'vodom', 'publish_tf': False,
                          'Odom/Strategy': '0', 'Vis/MinInliers': '12',
                          'Vis/MaxFeatures': '600', 'Odom/ResetCountdown': '1'}],
             remappings=CAM_REMAPS + [('odom', '/vo/odom')]),

        # RTAB-Map: camera odometry when visual_odom:=true ...
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap',
             output='screen', arguments=['-d'], condition=IfCondition(vo),
             parameters=[{**SLAM, 'subscribe_odom_info': False}],
             remappings=CAM_REMAPS + [('odom', '/vo/odom')]),
        # ... otherwise rf2o's odom TF as the motion prior (robust on plain walls)
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace='rtabmap',
             output='screen', arguments=['-d'], condition=UnlessCondition(vo),
             parameters=[{**SLAM, 'odom_frame_id': 'odom'}],
             remappings=CAM_REMAPS),
    ])
