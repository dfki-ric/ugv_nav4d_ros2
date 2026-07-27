#!/usr/bin/env python3
"""Interactive fake-posture publisher for footprint testing.

Broadcasts the wheel/tool TF frames the footprint measurement uses, driven by
node parameters with ranges -- open rqt_reconfigure and you get SLIDERS to
"move the robot links manually":

    ros2 run ugv_nav4d_ros2 posture_publisher.py
    ros2 run rqt_reconfigure rqt_reconfigure   # select footprint_posture_publisher

Then watch /ugv_nav4d_ros2/footprint_polygon + wheelbase markers in RViz
follow the sliders, and press "Update footprint" in the operator panel to
make the planner adopt the pose.

WARNING: conflicts with robot_state_publisher publishing the same frames --
bench use only (planner without a robot/sim on the same domain).
"""

import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

BASE = 'arter/base_link'


def ranged(desc, lo, hi, step=0.01):
    return ParameterDescriptor(
        description=desc,
        floating_point_range=[FloatingPointRange(from_value=lo, to_value=hi, step=step)])


class PosturePublisher(Node):
    def __init__(self):
        super().__init__('footprint_posture_publisher')
        self.declare_parameter('wheelbase', 4.86,
                               ranged('distance front<->rear wheel centers [m]', 2.0, 8.0))
        self.declare_parameter('track_width', 3.0,
                               ranged('distance left<->right wheel centers [m]', 1.0, 5.0))
        self.declare_parameter('wheelbase_center_x', 0.0,
                               ranged('x of the wheelbase midpoint relative to base_link [m]', -2.0, 2.0))
        self.declare_parameter('tool_x', 4.94,
                               ranged('tool_link x (boom extension) [m]', 0.0, 8.0))
        self.declare_parameter('tool_y', 0.0,
                               ranged('tool_link y (boom swing) [m]', -3.0, 3.0))
        # Fake localization: map -> base_link, so RViz (fixed frame map) can
        # display everything and the planner's pose lookup succeeds on the
        # bench. The sliders drive the robot around the map.
        self.declare_parameter('robot_x', 0.0,
                               ranged('robot x in map [m]', -50.0, 50.0))
        self.declare_parameter('robot_y', 0.0,
                               ranged('robot y in map [m]', -50.0, 50.0))
        self.declare_parameter('robot_yaw', 0.0,
                               ranged('robot yaw in map [rad]', -3.15, 3.15))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            'Publishing fake posture TF; use rqt_reconfigure sliders to move the links.')

    def tick(self):
        wheelbase = self.get_parameter('wheelbase').value
        track = self.get_parameter('track_width').value
        center = self.get_parameter('wheelbase_center_x').value
        tool_x = self.get_parameter('tool_x').value
        tool_y = self.get_parameter('tool_y').value

        frames = {
            'arter/wheel_fl_link': (center + wheelbase / 2.0, track / 2.0),
            'arter/wheel_fr_link': (center + wheelbase / 2.0, -track / 2.0),
            'arter/wheel_rl_link': (center - wheelbase / 2.0, track / 2.0),
            'arter/wheel_rr_link': (center - wheelbase / 2.0, -track / 2.0),
            'arter/tool_link': (tool_x, tool_y),
        }
        now = self.get_clock().now().to_msg()
        for frame, (x, y) in frames.items():
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = BASE
            tf.child_frame_id = frame
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

        pose = TransformStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.child_frame_id = BASE
        pose.transform.translation.x = float(self.get_parameter('robot_x').value)
        pose.transform.translation.y = float(self.get_parameter('robot_y').value)
        yaw = float(self.get_parameter('robot_yaw').value)
        pose.transform.rotation.z = math.sin(yaw / 2.0)
        pose.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(pose)


def main():
    rclpy.init()
    node = PosturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
