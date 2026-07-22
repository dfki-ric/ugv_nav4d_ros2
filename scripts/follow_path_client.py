#! /bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from ugv_nav4d_ros2.msg import LabeledPathArray, MissionStatus

import math

class FollowPathClient(Node):
    # A controller "success" with more than this much path length remaining is
    # treated as a false goal-checker trigger (drive-by near the goal pose) and
    # execution continues with the trimmed remainder.
    FALSE_SUCCESS_REMAINING_M = 5.0
    # Safety valve so a pathological mission cannot re-send forever.
    MAX_AUTO_CONTINUES = 10

    def __init__(self):
        super().__init__('follow_path_client')

        # Gated topic: the planner only publishes here after the operator confirms
        # the previewed path via the execute_path service / panel button.
        self.subscription = self.create_subscription(
            LabeledPathArray,
            '/ugv_nav4d_ros2/execute_path_segments',
            self.labeled_path_callback,
            10
        )
        self.combined_path_pub = self.create_publisher(
            Path, '/follow_path_client/combined_path', 10
        )

        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        # Operator stop: cancels the running FollowPath goal and drops all queued
        # segments. Named under the planner namespace for the RViz operator panel.
        self.stop_service = self.create_service(
            Trigger, '/ugv_nav4d_ros2/stop_execution', self.stop_callback)
        self.pause_service = self.create_service(
            Trigger, '/ugv_nav4d_ros2/pause_execution', self.pause_callback)
        self.resume_service = self.create_service(
            Trigger, '/ugv_nav4d_ros2/resume_execution', self.resume_callback)
        # Latched so a freshly opened operator panel immediately learns the
        # current state (e.g. PAUSED publishes nothing until resumed).
        latched_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            MissionStatus, '/ugv_nav4d_ros2/execution_status', latched_qos)
        self.route_valid_sub = self.create_subscription(
            Bool, '/ugv_nav4d_ros2/route_valid', self.route_valid_callback, 10)

        self.path_queue = []
        self.goal_in_progress = False
        self.current_goal_handle = None
        self.goal_finished = True
        self.pending_labeled_path_msg = None
        self.cancel_in_progress = False
        self.cancel_reason = None
        self.paused = False
        self.route_valid = True
        self.current_item = None
        self.current_segment = 0
        self.total_segments = 0
        self.distance_remaining = 0.0
        self.current_speed = 0.0
        self.auto_continue_count = 0

        self.get_logger().info('LabeledPath FollowPathClient ready.')
        self.publish_status(MissionStatus.READY, 'Follower ready')

    def publish_status(self, state, summary, failure_reason='', can_resume=None):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = state
        names = {
            MissionStatus.IDLE: 'IDLE', MissionStatus.READY: 'READY',
            MissionStatus.EXECUTING: 'EXECUTING', MissionStatus.PAUSED: 'PAUSED',
            MissionStatus.COMPLETED: 'COMPLETED', MissionStatus.FAILED: 'FAILED',
            MissionStatus.ABORTED: 'ABORTED'}
        msg.state_name = names.get(state, 'UNKNOWN')
        msg.summary = summary
        msg.failure_reason = failure_reason
        msg.current_segment = self.current_segment
        msg.total_segments = self.total_segments
        msg.progress = (float(self.current_segment - 1) / self.total_segments
                        if self.total_segments else 0.0)
        msg.distance_remaining = float(self.distance_remaining)
        msg.current_speed = float(self.current_speed)
        msg.estimated_seconds_remaining = (
            float(self.distance_remaining / abs(self.current_speed))
            if abs(self.current_speed) > 0.02 else -1.0)
        msg.motion_mode = self.current_item[1] if self.current_item else ''
        msg.route_valid = self.route_valid
        msg.can_resume = self.paused if can_resume is None else can_resume
        self.status_pub.publish(msg)

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
            self.cancel_reason = 'replace'
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
        self.paused = False
        self.route_valid = True
        self.current_item = None
        self.total_segments = len(self.path_queue)
        self.current_segment = 0
        self.auto_continue_count = 0
        self.send_next_path()

    def send_next_path(self):
        if not self.path_queue:
            self.get_logger().info('All paths executed.')
            self.goal_in_progress = False
            self.current_goal_handle = None
            self.goal_finished = True
            self.current_item = None
            self.publish_status(MissionStatus.COMPLETED, 'Mission completed')
            return

        self.current_item = self.path_queue.pop(0)
        self.current_segment += 1
        # Feedback from the previous segment must not leak into the resume
        # trimming of this one before the first feedback of this goal arrives.
        self.distance_remaining = 0.0
        self.send_current_path()

    def trimmed_current_path(self):
        """Return the current segment trimmed to the not-yet-driven remainder.

        Resume sends a brand-new FollowPath goal, and the controller searches
        for the robot only within max_robot_pose_search_dist of the path
        start. Re-sending the full segment makes it latch onto a pose behind
        the robot and briefly drive backwards. The last action feedback's
        distance_to_goal is the remaining path length, so keep only that much
        (plus a short tail so a slightly stale value still overlaps the
        robot). Falls back to the full path if no feedback arrived yet.
        """
        path, label = self.current_item
        remaining = float(self.distance_remaining)
        if remaining <= 0.0 or len(path.poses) < 3:
            return path

        keep = remaining + 0.5
        accumulated = 0.0
        start_idx = 0
        for i in range(len(path.poses) - 1, 0, -1):
            accumulated += self.pose_distance(path.poses[i - 1], path.poses[i])
            if accumulated >= keep:
                start_idx = i - 1
                break
        if start_idx == 0:
            return path

        trimmed = Path()
        trimmed.header = path.header
        trimmed.poses = path.poses[start_idx:]
        self.get_logger().info(
            f'Resume: trimmed already-driven part of segment "{label}", '
            f'keeping the last {keep:.1f} m '
            f'({len(trimmed.poses)}/{len(path.poses)} poses).')
        return trimmed

    def send_current_path(self, path_override=None):
        if self.current_item is None:
            self.publish_status(MissionStatus.FAILED, 'No segment available to resume',
                                'Internal execution queue is empty')
            return
        path, label = self.current_item
        if path_override is not None:
            path = path_override
        self.get_logger().info(f'Sending path labeled "{label}" with {len(path.poses)} poses.')

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = ''
        goal_msg.goal_checker_id = ''

        # Never block indefinitely inside a callback: this node is single-threaded,
        # so a missing /follow_path server would freeze ALL callbacks (including the
        # stop service). Fail fast with a clear error instead.
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                '/follow_path action server not available; dropping path. '
                'Is the controller running?')
            self.clear()
            # send_next_path() selects and numbers the first segment before the
            # controller availability check. No segment was actually submitted,
            # so report the mission as not started instead of misleadingly
            # displaying segment 1/N.
            self.current_segment = 0
            self.distance_remaining = 0.0
            self.current_speed = 0.0
            self.publish_status(MissionStatus.FAILED, 'Controller unavailable; mission not started',
                                '/follow_path action server is unavailable')
            return

        self.goal_in_progress = True
        self.goal_finished = False
        self.paused = False
        self.cancel_reason = None
        self.publish_status(
            MissionStatus.EXECUTING,
            f'Executing segment {self.current_segment}/{self.total_segments}: {label}')
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.distance_remaining = float(feedback.distance_to_goal)
        self.current_speed = float(feedback.speed)
        self.publish_status(
            MissionStatus.EXECUTING,
            f'Executing segment {self.current_segment}/{self.total_segments}')

    def route_valid_callback(self, msg):
        self.route_valid = bool(msg.data)
        if not self.route_valid and self.current_goal_handle is not None and not self.paused:
            self.get_logger().error('Remaining route invalidated; pausing execution.')
            self._request_pause('Route invalidated by a map or operational-zone change')

    def _request_pause(self, reason):
        if self.current_goal_handle is None or self.goal_finished:
            return False
        self.paused = True
        self.cancel_reason = 'pause'
        if not self.cancel_in_progress:
            self.cancel_in_progress = True
            future = self.current_goal_handle.cancel_goal_async()
            future.add_done_callback(self.after_cancel_pause)
        # Cancellation is asynchronous. Report PAUSED immediately for operator
        # awareness, but do not advertise resumability until Nav2 confirms the
        # cancellation callback below.
        self.publish_status(MissionStatus.PAUSED, reason, can_resume=False)
        return True

    def pause_callback(self, request, response):
        del request
        if self.paused:
            response.success = True
            response.message = 'Execution is already paused.'
        elif self._request_pause('Paused by operator'):
            response.success = True
            response.message = 'Pausing current segment; mission retained.'
        else:
            response.success = False
            response.message = 'Nothing is executing.'
        return response

    def after_cancel_pause(self, future):
        del future
        self.current_goal_handle = None
        self.goal_finished = True
        self.goal_in_progress = False
        self.cancel_in_progress = False
        self.current_speed = 0.0
        self.publish_status(MissionStatus.PAUSED, 'Paused; mission retained')

    def resume_callback(self, request, response):
        del request
        if not self.paused or self.current_item is None:
            response.success = False
            response.message = 'No paused mission is available.'
        elif not self.route_valid:
            response.success = False
            response.message = 'Route is invalid; replan before resuming.'
        else:
            response.success = True
            response.message = 'Resuming current segment.'
            self.send_current_path(self.trimmed_current_path())
        return response

    def stop_callback(self, request, response):
        # Clear the queue FIRST: a canceled goal reports status CANCELED, which
        # normally advances to the next queued segment (used when a new path
        # replaces a running one). With the queue empty, cancel means full stop.
        dropped = len(self.path_queue)
        self.path_queue.clear()
        self.paused = False
        self.cancel_reason = 'abort'
        if self.current_goal_handle is not None and not self.goal_finished:
            if not self.cancel_in_progress:
                self.cancel_in_progress = True
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.after_cancel_stop)
            response.success = True
            response.message = (
                f'Stopping: canceling current segment, dropped {dropped} queued segment(s).')
        else:
            self.goal_in_progress = False
            self.goal_finished = True
            self.current_goal_handle = None
            response.success = True
            response.message = 'Nothing executing; cleared queue.'
            self.current_item = None
            self.publish_status(MissionStatus.ABORTED, 'Mission aborted by operator')
        self.get_logger().warn(response.message)
        return response

    def after_cancel_stop(self, future):
        self.get_logger().warn('Execution stopped by operator.')
        self.current_goal_handle = None
        self.goal_finished = True
        self.cancel_in_progress = False
        self.current_item = None
        self.publish_status(MissionStatus.ABORTED, 'Mission aborted by operator')

    def clear(self):
        self.get_logger().error('Cleared internal state of FollowPath client.')
        self.path_queue.clear()
        self.goal_in_progress = False
        self.current_goal_handle = None
        self.goal_finished = True
        self.current_item = None
        self.paused = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal was rejected.')
            self.goal_in_progress = False
            self.goal_finished = True
            self.clear()
            self.publish_status(MissionStatus.FAILED, 'Controller rejected path',
                                'FollowPath goal was rejected')
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

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            # Nav2's goal checker only compares the robot pose against the final
            # goal pose; it fires on any drive-by near the goal (e.g. loop or
            # self-crossing routes) with most of the path still ahead. The action
            # feedback's distance_to_goal is remaining PATH LENGTH, so a success
            # with substantial length left is a false trigger: continue with the
            # trimmed remainder instead of completing the mission.
            if (self.current_item is not None and
                    self.distance_remaining > self.FALSE_SUCCESS_REMAINING_M and
                    self.auto_continue_count < self.MAX_AUTO_CONTINUES):
                self.auto_continue_count += 1
                self.get_logger().warn(
                    f'Controller reported success with {self.distance_remaining:.1f} m of '
                    f'path remaining; continuing the segment '
                    f'(auto-continue {self.auto_continue_count}/{self.MAX_AUTO_CONTINUES}).')
                self.send_current_path(self.trimmed_current_path())
                return
            self.get_logger().info('Goal completed, sending next if available.')
            self.current_item = None
            self.send_next_path()
        elif result.status == GoalStatus.STATUS_CANCELED:
            # Cancellation completion is handled by the callback that requested
            # pause/replace/abort. Never advance the queue merely because a goal
            # was canceled.
            self.get_logger().info(f'Goal canceled ({self.cancel_reason or "unspecified"}).')
        else:
            self.get_logger().warn(f'Goal failed with status: {result.status}. Stopping.')
            failure = ('FollowPath aborted' if result.status == GoalStatus.STATUS_ABORTED
                       else f'FollowPath ended with status {result.status}')
            self.clear()
            self.publish_status(MissionStatus.FAILED, 'Mission execution failed', failure)

def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
