# M1 guide: SLAM mapping workspace

Goal for Nov 8: one command (`ros2 launch slam_drone m1.launch.py fly:=true`) and the drone flies the corridor loop while slam_toolbox draws the floor plan live in RViz. Then save the map, record, and tag `m1`.

Tabs used here: **Tab 1** = simulation (PX4 + Gazebo), **Tab 2** = launch / ROS nodes, **Tab 3** = checks. The old Tab 2 bridge, Tab 4 static TFs and separate MicroXRCEAgent tab are **no longer needed**: `m1.launch.py` starts all of them.

---

## Step 0: install packages (Tab 3)

```bash
sudo apt update
sudo apt install -y git ros-jazzy-slam-toolbox ros-jazzy-ros-gz-bridge ros-jazzy-nav2-map-server ros-jazzy-tf2-tools
```

## Step 1: make ~/slam_drone a git repo + ROS 2 workspace (Tab 3)

Layout (your `m0/` backup stays where it is):

```
~/slam_drone/                 <- git repo + colcon workspace
  m0/                         <- existing backup
  maps/                       <- saved maps
  src/slam_drone/             <- ROS 2 package (ament_python)
    slam_drone/odom_tf.py         (this thread)
    slam_drone/waypoint_flight.py (from the "Waypoint offboard flight" thread)
    launch/m1.launch.py
    config/slam_toolbox_m1.yaml, config/m1.rviz
    sim/                      <- copies of model, world, airframe
```

1a. Folders and empty marker files:

```bash
cd ~/slam_drone
mkdir -p maps src/slam_drone/{slam_drone,launch,config,resource,sim}
touch src/slam_drone/resource/slam_drone src/slam_drone/slam_drone/__init__.py
```

1b. Paste each block below into Tab 3 as-is (each one writes one complete file).

**`package.xml`**

```bash
cat > ~/slam_drone/src/slam_drone/package.xml <<'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>slam_drone</name>
  <version>0.1.0</version>
  <description>SLAM-based autonomous drone for GPS-denied indoor navigation</description>
  <maintainer email="kishanrajballal22@gmail.com">Kishan Ballal</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>px4_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF
```

**`setup.cfg`**

```bash
cat > ~/slam_drone/src/slam_drone/setup.cfg <<'EOF'
[develop]
script_dir=$base/lib/slam_drone
[install]
install_scripts=$base/lib/slam_drone
EOF
```

**`setup.py`**

```bash
cat > ~/slam_drone/src/slam_drone/setup.py <<'EOF'
import os
from glob import glob
from setuptools import setup

package_name = 'slam_drone'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kishan Ballal',
    maintainer_email='kishanrajballal22@gmail.com',
    description='SLAM-based autonomous drone for GPS-denied indoor navigation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_tf = slam_drone.odom_tf:main',
            'waypoint_flight = slam_drone.waypoint_flight:main',
        ],
    },
)
EOF
```

**`slam_drone/odom_tf.py`**

```bash
cat > ~/slam_drone/src/slam_drone/slam_drone/odom_tf.py <<'EOF'
#!/usr/bin/env python3
"""PX4 odometry -> ROS odometry + TF (Milestone 1).

Subscribes to PX4's VehicleOdometry (local NED frame, body FRD),
converts it to ROS conventions (odom = ENU, base_link = FLU) and publishes:
  * TF  odom -> base_link
  * nav_msgs/Odometry on /odom

In M2 this node is switched off and rf2o laser odometry provides odom -> base_link.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def qmul(a, b):
    """Hamilton product of quaternions given as (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def qconj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    p = (0.0, v[0], v[1], v[2])
    r = qmul(qmul(q, p), qconj(q))
    return (r[1], r[2], r[3])


S = math.sqrt(0.5)
Q_NED_TO_ENU = (0.0, S, S, 0.0)   # swap x/y, flip z
Q_FRD_TO_FLU = (0.0, 1.0, 0.0, 0.0)  # 180 deg about x


def ned_to_enu(v):
    return (v[1], v[0], -v[2])


class OdomTf(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # PX4 publishes best-effort; VOLATILE subscriber is compatible with any durability.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(VehicleOdometry, topic, self.cb, px4_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.count = 0
        self.get_logger().info(f'Listening to {topic}, publishing {self.odom_frame} -> {self.base_frame} and /odom')

    def cb(self, msg: VehicleOdometry):
        # Ignore invalid samples (PX4 sends NaN before the EKF has a solution)
        if any(math.isnan(v) for v in list(msg.position) + list(msg.q)):
            return

        # Stamp with ROS time (= Gazebo sim time when use_sim_time is true),
        # so TF matches the /scan timestamps that SLAM uses.
        stamp = self.get_clock().now().to_msg()

        # Pose: NED/FRD -> ENU/FLU
        pos = ned_to_enu(msg.position)
        q_ned_frd = (float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3]))
        q = qmul(qmul(Q_NED_TO_ENU, q_ned_frd), Q_FRD_TO_FLU)
        n = math.sqrt(sum(c * c for c in q))
        q = tuple(c / n for c in q)

        # Linear velocity: PX4 gives it in NED (velocity_frame 1) or FRD body (3).
        # nav_msgs/Odometry wants it in the child frame (base_link, FLU).
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            vel_body = (msg.velocity[0], -msg.velocity[1], -msg.velocity[2])
        else:
            vel_body = rotate(qconj(q), ned_to_enu(msg.velocity))
        # Angular velocity: body FRD -> FLU
        w = msg.angular_velocity
        ang_body = (w[0], -w[1], -w[2])

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])
            t.transform.rotation.w = q[0]
            t.transform.rotation.x = q[1]
            t.transform.rotation.y = q[2]
            t.transform.rotation.z = q[3]
            self.tf_pub.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = float(pos[0])
        o.pose.pose.position.y = float(pos[1])
        o.pose.pose.position.z = float(pos[2])
        o.pose.pose.orientation.w = q[0]
        o.pose.pose.orientation.x = q[1]
        o.pose.pose.orientation.y = q[2]
        o.pose.pose.orientation.z = q[3]
        pv = [0.0 if math.isnan(v) else float(v) for v in msg.position_variance]
        ov = [0.0 if math.isnan(v) else float(v) for v in msg.orientation_variance]
        # NED variances -> ENU (swap x/y)
        o.pose.covariance[0] = pv[1]
        o.pose.covariance[7] = pv[0]
        o.pose.covariance[14] = pv[2]
        o.pose.covariance[21] = ov[0]
        o.pose.covariance[28] = ov[1]
        o.pose.covariance[35] = ov[2]
        o.twist.twist.linear.x = float(vel_body[0])
        o.twist.twist.linear.y = float(vel_body[1])
        o.twist.twist.linear.z = float(vel_body[2])
        o.twist.twist.angular.x = float(ang_body[0])
        o.twist.twist.angular.y = float(ang_body[1])
        o.twist.twist.angular.z = float(ang_body[2])
        self.odom_pub.publish(o)

        self.count += 1
        if self.count == 1:
            self.get_logger().info('First PX4 odometry received, TF is live')


def main(args=None):
    rclpy.init(args=args)
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

**`config/slam_toolbox_m1.yaml`**

```bash
cat > ~/slam_drone/src/slam_drone/config/slam_toolbox_m1.yaml <<'EOF'
# slam_toolbox online_async params for the x500_lidar_3d drone (Milestone 1)
slam_toolbox:
  ros__parameters:
    use_sim_time: true

    # Plugin params
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # ROS params
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02   # map->odom at 50 Hz
    map_update_interval: 1.0         # redraw map every 1 s (looks live in the video)
    resolution: 0.05
    min_laser_range: 0.3             # ignore the drone's own legs/props
    max_laser_range: 12.0
    minimum_time_interval: 0.2
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # General params
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # Correlation params
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    # Loop closure correlation params
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    # Scan matcher params
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1
EOF
```

**`config/m1.rviz`**

```bash
cat > ~/slam_drone/src/slam_drone/config/m1.rviz <<'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 1
      Plane Cell Count: 30
      Reference Frame: <Fixed Frame>
      Color: 160; 160; 164
    - Class: rviz_default_plugins/Map
      Name: Map
      Enabled: true
      Alpha: 0.8
      Color Scheme: map
      Draw Behind: true
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
    - Class: rviz_default_plugins/LaserScan
      Name: Scan
      Enabled: true
      Size (m): 0.05
      Style: Flat Squares
      Color Transformer: FlatColor
      Color: 255; 255; 255
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
    - Class: rviz_default_plugins/PointCloud2
      Name: Lidar3D
      Enabled: true
      Size (m): 0.02
      Style: Flat Squares
      Color Transformer: AxisColor
      Axis: Z
      Decay Time: 0
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /lidar_3d/points
    - Class: rviz_default_plugins/Odometry
      Name: Odometry
      Enabled: true
      Keep: 200
      Position Tolerance: 0.1
      Angle Tolerance: 0.3
      Shape:
        Value: Arrow
        Color: 255; 25; 0
        Shaft Length: 0.4
        Shaft Radius: 0.03
        Head Length: 0.15
        Head Radius: 0.08
      Covariance:
        Value: false
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Show Axes: true
      Show Arrows: false
      Marker Scale: 0.5
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Distance: 28
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 1.1
      Yaw: 4.71
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
    Saved: ~
Window Geometry:
  Height: 900
  Width: 1500
  X: 50
  Y: 50
EOF
```

**`launch/m1.launch.py`**

```bash
cat > ~/slam_drone/src/slam_drone/launch/m1.launch.py <<'EOF'
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

        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg],
             parameters=[SIM], condition=IfCondition(LaunchConfiguration('rviz'))),

        # Give SLAM and the bridges ~8 s to come up before the drone takes off.
        TimerAction(period=8.0, actions=[
            Node(package='slam_drone', executable='waypoint_flight', name='waypoint_flight',
                 output='screen', parameters=[SIM],
                 condition=IfCondition(LaunchConfiguration('fly')))]),
    ])
EOF
```

1c. Copy the M0 sim files into the repo (backup copies; PX4 still uses the originals):

```bash
cd ~/slam_drone/src/slam_drone/sim
cp -r ~/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_3d .
cp ~/PX4-Autopilot/Tools/simulation/gz/worlds/building.sdf .
cp ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4098_gz_x500_lidar_3d .
cp ~/drone.rviz .
```

1d. Until the waypoint node file arrives, `setup.py` points at a missing `waypoint_flight.py`. That is fine for building, but create a placeholder so nothing complains (the waypoint thread will give you the real file to overwrite it):

```bash
[ -f ~/slam_drone/src/slam_drone/slam_drone/waypoint_flight.py ] || printf 'def main():\n    print("waypoint_flight not installed yet")\n' > ~/slam_drone/src/slam_drone/slam_drone/waypoint_flight.py
```

1e. `.gitignore`, build, and source it automatically in new tabs:

```bash
cd ~/slam_drone
printf 'build/\ninstall/\nlog/\n__pycache__/\n*.pyc\n' > .gitignore
colcon build --symlink-install
echo 'source ~/slam_drone/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
ros2 pkg executables slam_drone
```

Expected: `slam_drone odom_tf` and `slam_drone waypoint_flight`.

1f. First commit:

```bash
cd ~/slam_drone
git init -b main
git config user.name "Kishan Ballal"
git config user.email "kishanrajballal22@gmail.com"
git add .
git commit -m "M1: ROS 2 package with odom_tf, slam_toolbox config, m1 launch"
git tag m0-frozen
```

📸 Send a photo of the `ros2 pkg executables` output and `git log --oneline`.

---

## Step 2: check clock and PX4 odometry topic names

**Tab 1** (as usual):

```bash
pkill -f "gz sim"; pkill -f px4
cd ~/PX4-Autopilot
PX4_GZ_WORLD=building make px4_sitl gz_x500_lidar_3d
```

Wait for `Ready for takeoff!`.

**Tab 2** (agent only for this check):

```bash
MicroXRCEAgent udp4 -p 8888
```

**Tab 3**:

```bash
gz topic -l | grep clock
ros2 topic list | grep -i odom
```

Expected: `/clock` in the first list, and something like `/fmu/out/vehicle_odometry` in the second. **If the odometry name has a suffix (e.g. `/fmu/out/vehicle_odometry_v1`)**, tell me; you'll just add `odom_topic:=/fmu/out/vehicle_odometry_v1` to every launch command below.

Then stop the agent in Tab 2 (Ctrl+C). Leave Tab 1 running.

📸 Send a photo of both outputs.

---

## Step 3 + 4 + 6: run M1 (mapping) with one command

**Tab 2** (PX4 already running in Tab 1):

```bash
ros2 launch slam_drone m1.launch.py
```

In the log, look for:
- `2D scan: /world/... /scan  frame: ...` and `3D cloud frame: ...` (auto-detected)
- `odom_tf: First PX4 odometry received, TF is live`
- slam_toolbox: `Registering sensor: [Custom Described Lidar]`

RViz opens with Fixed Frame `map`. Within a few seconds the grey/black floor plan of the area around the drone appears.

**Tab 1** at `pxh>`:

```
commander takeoff
```

**Tab 3** checks:

```bash
ros2 run tf2_ros tf2_echo odom base_link      # z should be ~1.5 after takeoff; Ctrl+C
ros2 topic hz /clock                          # should print a rate; Ctrl+C
ros2 run tf2_tools view_frames                # writes frames_*.pdf in current folder
```

The TF tree must be: `map -> odom -> base_link -> (2D lidar frame, 3D lidar frame)`.

📸 Send a photo of RViz with the map, and the tf2_echo output.

When done: `commander land` in Tab 1, Ctrl+C in Tab 2.

---

## Step 5 hook-up: autonomous loop

Once the waypoint thread's `waypoint_flight.py` is in `~/slam_drone/src/slam_drone/slam_drone/` (overwrite the placeholder), rebuild in **Tab 3**:

```bash
cd ~/slam_drone && colcon build --symlink-install
```

Then the full demo, PX4 freshly started in **Tab 1** (don't type `commander takeoff`, the node does it):

**Tab 2**:

```bash
ros2 launch slam_drone m1.launch.py fly:=true
```

The waypoint node starts 8 s after launch, takes off to 1.5 m, flies the loop (~75 s), lands. Watch the map close the loop when the drone comes back to (0, 0).

Start the screen recording (Win+Alt+R) before launching.

---

## Save the map + freeze M1 (Tab 3, while SLAM is still running)

```bash
cd ~/slam_drone/maps
ros2 run nav2_map_server map_saver_cli -f building_m1 --ros-args -p use_sim_time:=true
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/slam_drone/maps/building_m1'}"
ls
```

Expected files: `building_m1.pgm`, `building_m1.yaml`, `building_m1.posegraph`, `building_m1.data`.

```bash
cd ~/slam_drone
git add .
git commit -m "M1 complete: drone maps the building"
git tag m1
```

---

## If something breaks

| Symptom | Likely cause | What to do |
|---|---|---|
| `ERROR: no Gazebo topics found` | Launched before Tab 1 was up | Wait for `Ready for takeoff!`, relaunch Tab 2 |
| `could not read sensor frame ids` | Auto-detect failed | `ros2 topic echo /scan --field header.frame_id --once` (with an M0 bridge running), then add `scan_frame:=<value> cloud_frame:=<value>` |
| No `First PX4 odometry received` | Wrong odom topic name, or agent not running | Step 2 check; is another MicroXRCEAgent already open on port 8888? Close it |
| slam_toolbox: `Message Filter dropping message ... timestamp on the message is earlier than all the data in the transform cache` | A node is on wall time | Every node here sets use_sim_time; check `ros2 param get /odom_tf use_sim_time` → True |
| RViz: `No map received` | slam_toolbox didn't start or no TF | `ros2 node list` should show `/slam_toolbox`; `ros2 lifecycle get /slam_toolbox` should say active |
| Map smears/doubles when turning | Odometry rotation wrong | Send me a photo of `tf2_echo odom base_link` while the drone faces north (yaw should read ~90°) |

Send photos of anything red and I'll debug from there.
