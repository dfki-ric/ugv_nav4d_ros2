#! /bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from ugv_nav4d_ros2.msg import LabeledPathArray

import math

class FollowPathClient(Node):
    def __init__(self):
        super().__init__('follow_path_client')

        self.subscription = self.create_subscription(
            LabeledPathArray,
            '/ugv_nav4d_ros2/labeled_path_segments',
            self.labeled_path_callback,
            10
        )
        self.combined_path_pub = self.create_publisher(
            Path, '/follow_path_client/combined_path', 10
        )

        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        self.path_queue = []
        self.goal_in_progress = False
        self.current_goal_handle = None
        self.goal_finished = True
        self.pending_labeled_path_msg = None
        self.cancel_in_progress = False

        self.get_logger().info('LabeledPath FollowPathClient ready.')

    def pose_distance(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        dz = pose1.pose.position.z - pose2.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))

    def yaw_from_quaternion(self, q):
        """
        Extract yaw (rotation around Z) from a geometry_msgs quaternion.
        """
        # q has fields: x, y, z, w
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def filter_close_poses(self, poses, threshold=0.05):
        if len(poses) < 3:
            return poses

        filtered = [poses[0]]
        last_dir = None
        last_dyaw = None

        for pose in poses[1:]:
            prev = filtered[-1]

            dx = pose.pose.position.x - prev.pose.position.x
            dy = pose.pose.position.y - prev.pose.position.y
            dist = math.hypot(dx, dy)

            #print(dist)

            if dist < threshold:
                continue

            curr_dir = (dx / dist, dy / dist)

            if last_dir is not None:
                # dot product between directions
                dot = curr_dir[0] * last_dir[0] + curr_dir[1] * last_dir[1]

                # direction flipped or sharp reversal → skip
                if dot < 0.0:
                    continue

            filtered.append(pose)
            last_dir = curr_dir

        return filtered

    def labeled_path_callback(self, msg: LabeledPathArray):
        # Always combine and publish for visualization
        combined_path = Path()
        if msg.paths:
            combined_path.header = msg.paths[0].header
            last_pose = None
            for path in msg.paths:
                for pose in path.poses:
                    if last_pose is None or self.pose_distance(last_pose, pose) > 0.01:
                        combined_path.poses.append(pose)
                        last_pose = pose
            self.combined_path_pub.publish(combined_path)
            self.get_logger().info(f'Published combined path with {len(combined_path.poses)} poses.')
        else:
            self.get_logger().warn('No paths to combine in LabeledPathArray.')

        if self.current_goal_handle is not None and not self.goal_finished:
            self.get_logger().info("Canceling previous goal before accepting new one.")
            self.pending_labeled_path_msg = msg
            if not self.cancel_in_progress:
                self.cancel_in_progress = True
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.after_cancel_new_paths)
            return
        else:
            self.replace_queue_and_send(msg)

    def after_cancel_new_paths(self, future):
        self.get_logger().info("Previous goal canceled.")
        self.current_goal_handle = None
        self.goal_finished = True
        self.cancel_in_progress = False
        if self.pending_labeled_path_msg is not None:
            self.replace_queue_and_send(self.pending_labeled_path_msg)
            self.pending_labeled_path_msg = None

    def replace_queue_and_send(self, msg):
        self.path_queue.clear()
        min_dist_threshold = 0.01  # meters

        for path, label in zip(msg.paths, msg.labels):
            if not path.poses:
                self.get_logger().warn(f'Skipping empty path labeled "{label}".')
                continue

            filtered_poses = self.filter_close_poses(path.poses, min_dist_threshold)

            if not filtered_poses:
                self.get_logger().warn(f'All poses removed due to duplication in label "{label}". Skipping.')
                continue

            path.poses = filtered_poses
            self.path_queue.append((path, label))

        self.goal_in_progress = False
        self.goal_finished = True
        self.send_next_path()

    def send_next_path(self):
        if not self.path_queue:
            self.get_logger().info('All paths executed.')
            self.goal_in_progress = False
            self.current_goal_handle = None
            self.goal_finished = True
            return

        path, label = self.path_queue.pop(0)
        self.get_logger().info(f'Sending path labeled "{label}" with {len(path.poses)} poses.')

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = ''
        goal_msg.goal_checker_id = ''

        self.goal_in_progress = True
        self.goal_finished = False
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def clear(self):
        self.get_logger().error('Cleared internal state of FollowPath client.')
        self.path_queue.clear()
        self.goal_in_progress = False
        self.current_goal_handle = None
        self.goal_finished = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal was rejected.')
            self.goal_in_progress = False
            self.goal_finished = True
            self.clear()
            return

        self.get_logger().info('FollowPath goal accepted.')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Goal finished with status code: {result.status}")

        self.goal_finished = True
        self.goal_in_progress = False
        self.current_goal_handle = None

        if result.status in (2, 4):  # SUCCEEDED or CANCELED
            self.get_logger().info('Goal completed, sending next if available.')
            self.send_next_path()
        elif result.status == 6:  # UNKNOWN
            self.get_logger().warn('Received UNKNOWN status (6). Ignoring and waiting.')
        else:
            self.get_logger().warn(f'Goal failed with status: {result.status}. Stopping.')
            self.clear()

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
