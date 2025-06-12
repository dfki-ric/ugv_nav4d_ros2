import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from ugv_nav4d_ros2.msg import LabeledPathArray

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
        self.goal_finished = True  # True if no goal is running
        self.pending_labeled_path_msg = None
        self.cancel_in_progress = False

        self.get_logger().info('LabeledPath FollowPathClient ready.')

    def labeled_path_callback(self, msg: LabeledPathArray):
        # Always combine and publish for visualization
        combined_path = Path()
        if msg.paths:
            combined_path.header = msg.paths[0].header
            for path in msg.paths:
                combined_path.poses.extend(path.poses)
            self.combined_path_pub.publish(combined_path)
            self.get_logger().info(f'Published combined path with {len(combined_path.poses)} poses.')
        else:
            self.get_logger().warn('No paths to combine in LabeledPathArray.')

        # If a goal is in progress, cancel it and buffer the new message
        if self.current_goal_handle is not None and not self.goal_finished:
            self.get_logger().info("Canceling previous goal before accepting new one.")
            self.pending_labeled_path_msg = msg   # Buffer the latest incoming request
            if not self.cancel_in_progress:
                self.cancel_in_progress = True
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.after_cancel_new_paths)
            return
        else:
            self.replace_queue_and_send(msg)

    def after_cancel_new_paths(self, future):
        # This callback is called after a goal is canceled.
        self.get_logger().info("Previous goal canceled.")
        self.current_goal_handle = None
        self.goal_finished = True
        self.cancel_in_progress = False
        # Now process the buffered request, if any
        if self.pending_labeled_path_msg is not None:
            self.replace_queue_and_send(self.pending_labeled_path_msg)
            self.pending_labeled_path_msg = None

    def replace_queue_and_send(self, msg):
        self.path_queue.clear()
        for path, label in zip(msg.paths, msg.labels):
            if not path.poses:
                self.get_logger().warn(f'Skipping empty path labeled "{label}".')
                continue
            self.path_queue.append((path, label))
        self.goal_in_progress = False
        self.goal_finished = True  # Safe to send next path
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

        # Accept SUCCEEDED or CANCELED as "done" and proceed
        if result.status in (2, 4):
            self.get_logger().info('Goal completed (succeeded or canceled), sending next if available.')
            self.send_next_path()
        elif result.status == 6:
            # UNKNOWN status - ignore it, just log a warning and do not clear state!
            self.get_logger().warn('Received UNKNOWN status (6). Ignoring and waiting for proper status.')
        else:
            self.get_logger().warn(f'Goal finished with non-success status: {result.status}. Stopping.')
            self.clear()

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
