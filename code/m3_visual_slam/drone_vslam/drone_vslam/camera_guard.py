"""camera_guard: runs the camera bridge only while memory is safe.

Gazebo renders a camera only while someone subscribes to it. This node owns
the ros_gz_bridge process for the camera topics, so:
  * bridge running  -> Gazebo renders the camera -> RTAB-Map gets images
  * bridge stopped  -> nobody subscribes -> Gazebo stops rendering it

Every `check_period` s it reads (from /proc, no extra packages):
  * the resident memory of all Gazebo (gz sim) processes
  * MemAvailable of the machine (WSL VM)
and STOPS the camera when gz RSS grew more than `max_growth_mb` since the
camera started, gz RSS > `max_gz_mb`, free memory < `min_free_mb`, or the
camera ran for `max_minutes`. The flight, LiDAR SLAM and Nav2 keep going.
Status (one line JSON) on /camera_guard/status, and a CSV log in ~/slam_drone/logs.

Restart the camera later with:  ros2 service call /camera_guard/start std_srvs/srv/Trigger
Stop it by hand with:           ros2 service call /camera_guard/stop std_srvs/srv/Trigger
"""
import json
import os
import signal
import subprocess
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

BRIDGE_TOPICS = [
    '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
    '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
]


def gz_rss_mb():
    """Sum of RSS of processes whose command line contains 'gz sim' (MB)."""
    total = 0
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ')
            if b'gz sim' not in cmd and b'gz-sim' not in cmd:
                continue
            with open(f'/proc/{pid}/status') as f:
                for line in f:
                    if line.startswith('VmRSS:'):
                        total += int(line.split()[1])
                        break
        except (OSError, ValueError):
            continue
    return total / 1024.0


def mem_available_mb():
    with open('/proc/meminfo') as f:
        for line in f:
            if line.startswith('MemAvailable:'):
                return int(line.split()[1]) / 1024.0
    return -1.0


class CameraGuard(Node):
    def __init__(self):
        super().__init__('camera_guard')
        self.declare_parameter('max_growth_mb', 1500.0)
        self.declare_parameter('max_gz_mb', 7000.0)
        self.declare_parameter('min_free_mb', 2500.0)
        self.declare_parameter('max_minutes', 10.0)
        self.declare_parameter('check_period', 5.0)
        self.declare_parameter('autostart', True)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.lim = {k: float(g(k)) for k in ('max_growth_mb', 'max_gz_mb', 'min_free_mb',
                                             'max_minutes')}
        self.proc = None
        self.t_on = None
        self.base_rss = None
        self.pub = self.create_publisher(String, '/camera_guard/status', 1)
        self.create_service(Trigger, '/camera_guard/start', self.srv_start)
        self.create_service(Trigger, '/camera_guard/stop', self.srv_stop)
        logdir = os.path.expanduser('~/slam_drone/logs')
        os.makedirs(logdir, exist_ok=True)
        self.log = open(os.path.join(logdir, time.strftime('camera_mem_%Y%m%d_%H%M%S.csv')), 'w')
        self.log.write('wall_s,camera_on,gz_rss_mb,free_mb\n')
        self.t0 = time.time()
        # wall-clock timer: must keep working even if the sim clock stalls
        self.timer = self.create_timer(float(g('check_period')), self.check,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        if g('autostart'):
            self.start('autostart')

    def start(self, why):
        if self.proc is not None and self.proc.poll() is None:
            return False
        cmd = ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', *BRIDGE_TOPICS,
               '--ros-args', '-r', '__node:=camera_bridge', '-p', 'use_sim_time:=true']
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        self.t_on = time.time()
        self.base_rss = gz_rss_mb()
        self.get_logger().info(f'CAMERA ON ({why}); gz now {self.base_rss:.0f} MB')
        return True

    def stop(self, why):
        if self.proc is None or self.proc.poll() is not None:
            self.proc = None
            return False
        os.killpg(self.proc.pid, signal.SIGINT)
        try:
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(self.proc.pid, signal.SIGKILL)
        self.proc = None
        self.get_logger().warn(f'CAMERA OFF: {why}. LiDAR SLAM and flight continue.')
        return True

    def srv_start(self, _req, res):
        res.success = self.start('service')
        res.message = 'started' if res.success else 'already running'
        return res

    def srv_stop(self, _req, res):
        res.success = self.stop('stopped by service')
        res.message = 'stopped' if res.success else 'was not running'
        return res

    def check(self):
        rss, free = gz_rss_mb(), mem_available_mb()
        on = self.proc is not None and self.proc.poll() is None
        st = {'camera_on': on, 'gz_rss_mb': round(rss), 'free_mb': round(free)}
        if on:
            growth = rss - (self.base_rss or rss)
            minutes = (time.time() - self.t_on) / 60.0
            st.update(growth_mb=round(growth), on_minutes=round(minutes, 1))
            if growth > self.lim['max_growth_mb']:
                self.stop(f'Gazebo memory grew {growth:.0f} MB since camera on')
            elif rss > self.lim['max_gz_mb']:
                self.stop(f'Gazebo uses {rss:.0f} MB')
            elif 0 <= free < self.lim['min_free_mb']:
                self.stop(f'only {free:.0f} MB free')
            elif minutes > self.lim['max_minutes']:
                self.stop(f'time limit {self.lim["max_minutes"]:.0f} min')
        self.pub.publish(String(data=json.dumps(st)))
        self.log.write(f'{time.time() - self.t0:.0f},{int(on)},{rss:.0f},{free:.0f}\n')
        self.log.flush()

    def destroy_node(self):
        self.stop('shutdown')
        self.log.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraGuard()
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
