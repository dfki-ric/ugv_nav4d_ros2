import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from ugv_nav4d_ros2.msg import LabeledPathArray  # Use your actual package name

class FollowPathClient(Node):
    def __init__(self):
        super().__init__('follow_path_client')

        self.subscription = self.create_subscription(
            LabeledPathArray,
            '/ugv_nav4d_ros2/labeled_paths',
            self.labeled_path_callback,
            10
        )

        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        self.path_queue = []
        self.goal_in_progress = False

        self.get_logger().info('LabeledPath FollowPathClient ready.')

    def labeled_path_callback(self, msg: LabeledPathArray):
        if len(msg.paths) != len(msg.labels):
            self.get_logger().error('Mismatch between paths and labels. Ignoring message.')
            return

        self.get_logger().info(f'Received {len(msg.paths)} labeled paths.')

        for path, label in zip(msg.paths, msg.labels):
            if not path.poses:
                self.get_logger().warn(f'Skipping empty path labeled "{label}".')
                continue
            self.path_queue.append((path, label))

        if not self.goal_in_progress:
            self._send_next_path()

    def _send_next_path(self):
        if not self.path_queue:
            self.get_logger().info('All paths executed.')
            self.goal_in_progress = False
            return

        path, label = self.path_queue.pop(0)
        self.get_logger().info(f'Sending path labeled "{label}" with {len(path.poses)} poses.')

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = ''
        goal_msg.goal_checker_id = ''

        self.goal_in_progress = True
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal was rejected.')
            self.goal_in_progress = False
            self._send_next_path()
            return

        self.get_logger().info('FollowPath goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'FollowPath result received: success = {result}')
        self.goal_in_progress = False
        self._send_next_path()


def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
