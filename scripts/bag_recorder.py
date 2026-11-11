#! /bin/env python3
"""Service-driven rosbag recorder for the navigation stack.

Runs ON the robot (next to the planner), so recording never loads the
operator link. The operator panel toggles it via two Trigger services:

    /ugv_nav4d_ros2/start_bag_recording
    /ugv_nav4d_ros2/stop_bag_recording

State is published latched on /ugv_nav4d_ros2/bag_recorder_status, so a
freshly opened panel immediately shows whether a recording is running.

The topic selection is regex-based: everything under the planner and
follow-path namespaces plus TF and the pose/command sources. The input
point cloud is included by default (it is what map bugs need for
reproduction) and can be disabled via the record_pointcloud parameter
when disk space is tight.
"""

import datetime
import os
import shutil
import signal
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

TOPIC_REGEX = (
    '/ugv_nav4d_ros2/.*'
    '|/follow_path_client/.*'
    '|/follow_path/_action/.*'
)
EXPLICIT_TOPICS = [
    '/tf',
    '/tf_static',
    '/arter/mcs/cmd_vel',
    '/arter/state/gnss_odometry',
    '/arter/state/kinematic_odometry',
    '/arter/state/wheel_odometry',
    '/rosout',
]


class BagRecorder(Node):
    def __init__(self):
        super().__init__('nav_bag_recorder')
        self.declare_parameter('output_dir', '/opt/workspace/bags')
        self.declare_parameter('record_pointcloud', True)
        self.declare_parameter('pointcloud_topic', '/ugv_nav4d_ros2/pointcloud')

        self.process = None
        self.bag_path = ''
        self.started_at = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            String, '/ugv_nav4d_ros2/bag_recorder_status', latched)
        self.create_service(
            Trigger, '/ugv_nav4d_ros2/start_bag_recording', self.start_cb)
        self.create_service(
            Trigger, '/ugv_nav4d_ros2/stop_bag_recording', self.stop_cb)
        # Watch the child: publish size/duration while running, detect crashes.
        self.create_timer(2.0, self.watch)
        self.publish_status('idle')
        self.get_logger().info('nav bag recorder ready.')

    # ------------------------------------------------------------------
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def bag_size_mb(self):
        total = 0
        if self.bag_path and os.path.isdir(self.bag_path):
            for name in os.listdir(self.bag_path):
                total += os.path.getsize(os.path.join(self.bag_path, name))
        return total / 1e6

    def running(self):
        return self.process is not None and self.process.poll() is None

    # ------------------------------------------------------------------
    def start_cb(self, request, response):
        if self.running():
            response.success = False
            response.message = f'Already recording: {self.bag_path}'
            return response

        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        free_gb = shutil.disk_usage(out_dir).free / 1e9
        if free_gb < 2.0:
            response.success = False
            response.message = (
                f'Refusing to record: only {free_gb:.1f} GB free in {out_dir}.')
            self.publish_status(f'idle | {response.message}')
            return response

        stamp = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
        self.bag_path = os.path.join(out_dir, f'nav_{stamp}')

        cmd = ['ros2', 'bag', 'record', '-o', self.bag_path,
               '--regex', TOPIC_REGEX] + EXPLICIT_TOPICS
        if not self.get_parameter('record_pointcloud').value:
            cmd += ['--exclude-topics',
                    self.get_parameter('pointcloud_topic').value]

        # Own process group so stop can signal the whole ros2-bag tree.
        self.process = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid)
        self.started_at = self.get_clock().now()
        response.success = True
        response.message = f'Recording to {self.bag_path} ({free_gb:.0f} GB free)'
        self.publish_status(f'RECORDING {os.path.basename(self.bag_path)} | 0 MB')
        self.get_logger().info(response.message)
        return response

    def stop_cb(self, request, response):
        if not self.running():
            self.process = None
            response.success = False
            response.message = 'Not recording.'
            self.publish_status('idle')
            return response

        # SIGINT lets ros2 bag close the mcap cleanly; SIGKILL would corrupt it.
        os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        try:
            self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=5)
        size = self.bag_size_mb()
        response.success = True
        response.message = f'Recording stopped: {self.bag_path} ({size:.0f} MB)'
        self.publish_status(f'idle | last bag: '
                            f'{os.path.basename(self.bag_path)} ({size:.0f} MB)')
        self.get_logger().info(response.message)
        self.process = None
        return response

    # ------------------------------------------------------------------
    def watch(self):
        if self.process is None:
            return
        if self.process.poll() is not None:
            # Child ended without a stop call: crashed or disk full.
            code = self.process.returncode
            self.publish_status(
                f'idle | recorder EXITED (code {code}), check disk space; '
                f'last bag: {os.path.basename(self.bag_path)}')
            self.get_logger().error(f'ros2 bag record exited with code {code}.')
            self.process = None
            return
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        self.publish_status(
            f'RECORDING {os.path.basename(self.bag_path)} | '
            f'{self.bag_size_mb():.0f} MB | {int(elapsed) // 60}:{int(elapsed) % 60:02d}')


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorder()
    try:
        rclpy.spin(node)
    finally:
        if node.running():
            os.killpg(os.getpgid(node.process.pid), signal.SIGINT)
            node.process.wait(timeout=15)
        node.destroy_node()


if __name__ == '__main__':
    main()
