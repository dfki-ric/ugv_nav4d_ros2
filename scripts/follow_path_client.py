#! /bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from ugv_nav4d_ros2.msg import LabeledPathArray, MissionStatus

import datetime
import json
import math
import os
import time

class FollowPathClient(Node):
    # A controller "success" with more than this much path length remaining is
    # treated as a false goal-checker trigger (drive-by near the goal pose) and
    # execution continues with the trimmed remainder.
    FALSE_SUCCESS_REMAINING_M = 5.0
    # Safety valve so a pathological mission cannot re-send forever.
    MAX_AUTO_CONTINUES = 10
    # Nav2's RPP controller never aborts on cross-track error (it keeps
    # steering toward the carrot however far the robot drifts), and the
    # SimpleProgressChecker only fires when the robot stops moving — so a
    # diverging robot drives away unchecked. This watchdog pauses execution
    # (cancel + zero velocity, mission retained) once the robot is farther
    # than this from the current segment.
    # Defaults for the max_path_deviation_m / deviation_breaches_to_pause
    # PARAMETERS (tunable per deployment and live via `ros2 param set`).
    MAX_PATH_DEVIATION_M = 2.0
    # Consecutive breached checks (at DEVIATION_CHECK_PERIOD_S) required
    # before pausing, so a single localization jump cannot stop a mission.
    DEVIATION_BREACHES_TO_PAUSE = 2
    DEVIATION_CHECK_PERIOD_S = 0.5
    # Pause-at-waypoints trigger distance (XY) plus a z window so a waypoint
    # on another level (ramp above, floor below) can never trigger the pause:
    # waypoint z and robot z share the body-frame height convention, so on
    # the same level they agree within slope/calibration error, while other
    # storeys differ by meters.
    # Pause-at-waypoints fires at the moment of REACHING the waypoint: the
    # distance is tracked once the robot is inside the arm radius, and the
    # pause triggers when the robot is on the point (AT radius) or the
    # distance starts growing again (closest approach passed). The machine
    # then brakes and rests just past the waypoint.
    WAYPOINT_ARM_RADIUS_M = 3.0
    WAYPOINT_AT_RADIUS_M = 0.4
    WAYPOINT_RECEDE_M = 0.3
    # Receding only counts as "reached" if the closest approach actually got
    # near the point; merely brushing the arm radius and turning away (e.g.
    # a direction change 2.5 m from the waypoint) must not fire.
    WAYPOINT_REACH_MIN_M = 0.8
    # Shared same-level window: poses/waypoints and the robot both carry the
    # body-frame height convention, so same level agrees within slope and
    # calibration error while other storeys differ by meters.
    LEVEL_Z_WINDOW_M = 2.0
    ROBOT_FRAME = 'arter/base_link'

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
        # Backstop for the operator Stop button: a zeroed CancelGoal request on
        # the action's cancel service cancels ALL goals the controller holds,
        # even one this client lost track of.
        self.cancel_all_client = self.create_client(
            CancelGoal, '/follow_path/_action/cancel_goal')

        # Operator stop: cancels the running FollowPath goal and drops all queued
        # segments. Named under the planner namespace for the RViz operator panel.
        # Pause-at-waypoints: when enabled, execution pauses once near each
        # queued waypoint and the operator must press Resume. Only meaningful
        # for waypoint routes; a single-goal route has an empty queue.
        self.pause_at_waypoints = False
        self.waypoint_xyz = []
        self.waypoints_paused = set()
        # Waypoints sitting at the END of the current segment (direction
        # change at the waypoint): those pause AFTER the segment completes,
        # so the controller drives its full (extended) trajectory and stops
        # AT the waypoint instead of being canceled 1.5 m short.
        self.waypoint_min_dist = {}
        # Waypoint photos: on every waypoint pause the latest camera frame is
        # written to disk; the index->path map is published latched so the
        # planner persists it into the mission file on save_mission.
        self.declare_parameter('waypoint_photo_topic',
                               '/arter/prosilica_left/image_raw/compressed')
        self.declare_parameter('waypoint_photo_dir', '/opt/workspace/missions/photos')
        self.latest_photo_msg = None
        self.latest_photo_wall = 0.0
        self.waypoint_photos = {}
        self.create_subscription(
            CompressedImage,
            self.get_parameter('waypoint_photo_topic').value,
            self.photo_frame_callback, 1)
        self.waypoint_photos_pub = self.create_publisher(
            String, '/follow_path_client/waypoint_photos',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.waypoint_photos_pub.publish(String(data='{}'))
        self.pause_at_wp_state_pub = self.create_publisher(
            Bool, '/follow_path_client/pause_at_waypoints',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pause_at_wp_state_pub.publish(Bool(data=False))
        self.create_service(
            SetBool, '/ugv_nav4d_ros2/set_pause_at_waypoints',
            self.set_pause_at_waypoints_callback)
        self.create_subscription(
            PoseArray, '/ugv_nav4d_ros2/waypoint_poses',
            self.waypoint_poses_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

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
        # Incremented on every goal send. Nav2 terminates a preempted goal as
        # ABORTED (not CANCELED), and that late result must not clobber the
        # state of the replacement goal that is already executing — otherwise
        # Stop sees "nothing executing" while the robot keeps driving.
        self.goal_generation = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('max_path_deviation_m', self.MAX_PATH_DEVIATION_M)
        self.declare_parameter('deviation_breaches_to_pause', self.DEVIATION_BREACHES_TO_PAUSE)
        # Live deviation readout for RViz: text + a line from the robot to the
        # nearest path point, colored by closeness to the pause limit.
        self.deviation_marker_pub = self.create_publisher(
            MarkerArray, '/follow_path_client/path_deviation_markers', 10)
        self.deviation_text_pub = self.create_publisher(
            String, '/follow_path_client/deviation_text', 10)
        self.deviation_pub = self.create_publisher(
            Float32, '/follow_path_client/path_deviation', 10)
        self.deviation_markers_active = False
        self.deviation_breaches = 0
        self.deviation_timer = self.create_timer(
            self.DEVIATION_CHECK_PERIOD_S, self.check_path_deviation)

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
        if not self.cancel_in_progress:
            return  # already finalized via the goal result
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
        self.waypoints_paused = set()
        self.waypoint_min_dist = {}
        self.waypoint_photos = {}
        self.waypoint_photos_pub.publish(String(data='{}'))
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
        self.goal_generation += 1
        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(
            lambda f, gen=self.goal_generation: self.goal_response_callback(f, gen))

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

    def clear_deviation_markers(self):
        if not self.deviation_markers_active:
            return
        self.deviation_markers_active = False
        idle = String()
        idle.data = 'idle'
        self.deviation_text_pub.publish(idle)
        clear = Marker()
        clear.action = Marker.DELETEALL
        msg = MarkerArray()
        msg.markers.append(clear)
        self.deviation_marker_pub.publish(msg)

    def publish_deviation_markers(self, frame_id, rx, ry, rz, nearest, deviation, limit):
        markers = MarkerArray()
        ratio = deviation / limit if limit > 0.0 else 0.0
        if ratio < 0.5:
            r, g, b = 0.2, 1.0, 0.3
        elif ratio < 1.0:
            r, g, b = 1.0, 0.8, 0.0
        else:
            r, g, b = 1.0, 0.2, 0.2

        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'path_deviation'
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.05
        line.color.r, line.color.g, line.color.b, line.color.a = r, g, b, 0.9
        start = type(nearest.pose.position)()
        start.x, start.y, start.z = rx, ry, rz + 0.1
        end = type(nearest.pose.position)()
        end.x = nearest.pose.position.x
        end.y = nearest.pose.position.y
        end.z = nearest.pose.position.z + 0.1
        line.points.append(start)
        line.points.append(end)
        markers.markers.append(line)

        # The numeric readout lives in the operator panel (deviation_text
        # topic); only the robot-to-path line stays in the 3D view. The
        # DELETEs clear the old text markers in running rviz sessions.
        for stale_ns, stale_id in (('path_deviation', 1), ('path_deviation_big', 2)):
            stale = Marker()
            stale.header.frame_id = frame_id
            stale.header.stamp = line.header.stamp
            stale.ns = stale_ns
            stale.id = stale_id
            stale.action = Marker.DELETE
            markers.markers.append(stale)

        self.deviation_marker_pub.publish(markers)
        self.deviation_markers_active = True

        text_msg = String()
        text_msg.data = f'{deviation:.2f} / {limit:.1f} m'
        self.deviation_text_pub.publish(text_msg)

    def check_path_deviation(self):
        if (self.current_item is None or self.goal_finished or self.paused
                or self.cancel_in_progress or self.current_goal_handle is None):
            self.deviation_breaches = 0
            self.clear_deviation_markers()
            return

        path, label = self.current_item
        if not path.poses or not path.header.frame_id:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                path.header.frame_id, self.ROBOT_FRAME, Time())
        except Exception:
            # No localization yet; readiness monitoring reports that separately.
            return

        limit = float(self.get_parameter('max_path_deviation_m').value)
        breaches_to_pause = max(1, int(self.get_parameter('deviation_breaches_to_pause').value))

        # XY only: path z carries the body-frame ground-clearance convention,
        # which would inflate a 3D distance without any real divergence.
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        rz = tf.transform.translation.z
        if self.check_waypoint_pause(rx, ry, rz):
            return
        # Same-level poses only: on a route that overlaps itself vertically
        # (ramp, underpass of its own path), the XY-nearest pose can lie on
        # another storey and mask a real divergence. Fall back to the full
        # path if the filter empties (degenerate z data).
        level_poses = [p for p in path.poses
                       if abs(p.pose.position.z - rz) <= self.LEVEL_Z_WINDOW_M]
        nearest = min(
            level_poses or path.poses,
            key=lambda p: (p.pose.position.x - rx) ** 2 + (p.pose.position.y - ry) ** 2)
        deviation = math.hypot(nearest.pose.position.x - rx, nearest.pose.position.y - ry)

        self.deviation_pub.publish(Float32(data=float(deviation)))
        self.publish_deviation_markers(path.header.frame_id, rx, ry, rz, nearest, deviation, limit)

        if deviation <= limit:
            self.deviation_breaches = 0
            return
        self.deviation_breaches += 1
        if self.deviation_breaches < breaches_to_pause:
            return
        self.deviation_breaches = 0
        self.get_logger().error(
            f'Robot is {deviation:.1f} m from segment "{label}" '
            f'(limit {limit} m); pausing execution.')
        self._request_pause(
            f'Robot diverged {deviation:.1f} m from the path '
            f'(limit {limit} m); replan from the robot position')

    def set_pause_at_waypoints_callback(self, request, response):
        self.pause_at_waypoints = bool(request.data)
        self.pause_at_wp_state_pub.publish(Bool(data=self.pause_at_waypoints))
        response.success = True
        response.message = ('Pause at each waypoint ENABLED; press Resume after each stop.'
                            if self.pause_at_waypoints else 'Pause at waypoints disabled.')
        self.get_logger().info(response.message)
        return response

    def waypoint_poses_callback(self, msg):
        self.waypoint_xyz = [(p.position.x, p.position.y, p.position.z)
                             for p in msg.poses]
        # Queue changed: earlier stops no longer map to the same waypoints.
        self.waypoints_paused = set()
        self.waypoint_min_dist = {}

    def photo_frame_callback(self, msg):
        self.latest_photo_msg = msg
        self.latest_photo_wall = time.time()

    def capture_waypoint_photo(self, wp_index):
        """Write the latest camera frame for this waypoint; never raises."""
        try:
            if self.latest_photo_msg is None:
                self.get_logger().warn(
                    f'Waypoint {wp_index + 1}: no camera frame received on '
                    f'{self.get_parameter("waypoint_photo_topic").value}; no photo saved.')
                return
            age = time.time() - self.latest_photo_wall
            if age > 5.0:
                self.get_logger().warn(
                    f'Waypoint {wp_index + 1}: newest camera frame is {age:.0f} s old; '
                    f'saving it anyway.')
            photo_dir = self.get_parameter('waypoint_photo_dir').value
            os.makedirs(photo_dir, exist_ok=True)
            ext = 'jpg' if 'jp' in (self.latest_photo_msg.format or '').lower() else 'png'
            stamp = datetime.datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
            path = os.path.join(photo_dir, f'wp{wp_index + 1}_{stamp}.{ext}')
            with open(path, 'wb') as f:
                f.write(bytes(self.latest_photo_msg.data))
            self.waypoint_photos[str(wp_index + 1)] = path
            self.waypoint_photos_pub.publish(
                String(data=json.dumps(self.waypoint_photos)))
            self.get_logger().info(f'Waypoint {wp_index + 1}: photo saved to {path}')
        except Exception as e:  # noqa: BLE001 -- a photo must never break driving
            self.get_logger().error(f'Waypoint photo capture failed: {e}')

    def check_waypoint_pause(self, rx, ry, rz):
        """Pause once per waypoint at the moment it is REACHED (closest
        approach), same level only. Returns True if a pause was just
        requested (skip further checks this tick)."""
        if not self.pause_at_waypoints or not self.waypoint_xyz:
            return False
        # STRICT ORDER: waypoints are visited in queue order (the planner
        # routes through them in sequence), so only the next un-paused
        # waypoint is ever eligible. Driving past waypoint 2's location on
        # the way to waypoint 1 must not pause.
        pending = [i for i in range(len(self.waypoint_xyz))
                   if i not in self.waypoints_paused]
        if not pending:
            return False
        for idx in (min(pending),):
            wx, wy, wz = self.waypoint_xyz[idx]
            if abs(wz - rz) > self.LEVEL_Z_WINDOW_M:
                continue
            d = math.hypot(wx - rx, wy - ry)
            if d > self.WAYPOINT_ARM_RADIUS_M:
                continue
            seen_min = self.waypoint_min_dist.get(idx)
            self.waypoint_min_dist[idx] = d if seen_min is None else min(seen_min, d)
            reached = (d <= self.WAYPOINT_AT_RADIUS_M or
                       (seen_min is not None and
                        seen_min <= self.WAYPOINT_REACH_MIN_M and
                        d >= seen_min + self.WAYPOINT_RECEDE_M))
            if not reached:
                continue
            # A pause can only take hold while a goal is active; in the gap
            # between two segments it fails and is retried on the next tick.
            if self._request_pause(
                    f'Waypoint {idx + 1} reached; press Resume to continue'):
                self.waypoints_paused.add(idx)
                self.waypoint_min_dist.pop(idx, None)
                self.capture_waypoint_photo(idx)
                self.get_logger().info(
                    f'Waypoint {idx + 1} reached; pausing (pause-at-waypoints is on).')
                return True
        return False

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
        if not self.cancel_in_progress:
            return  # already finalized via the goal result
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
        self.waypoint_min_dist = {}
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
            # Stop must stop the robot even if this client believes nothing is
            # running: cancel every goal the controller holds.
            if self.cancel_all_client.service_is_ready():
                self.cancel_all_client.call_async(CancelGoal.Request())
            self.publish_status(MissionStatus.ABORTED, 'Mission aborted by operator')
        self.get_logger().warn(response.message)
        return response

    def after_cancel_stop(self, future):
        if not self.cancel_in_progress:
            return  # already finalized via the goal result
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

    def goal_response_callback(self, future, generation):
        # An exception escaping this callback would kill the follower node (and
        # the operator's Stop service with it) while the robot may be driving.
        try:
            self._goal_response(future, generation)
        except Exception as e:  # noqa: BLE001 — last-resort containment
            self.get_logger().error(f'goal_response_callback failed: {e}')
            self.clear()
            self.publish_status(MissionStatus.FAILED, 'Mission execution failed',
                                f'Internal error in goal response handling: {e}')

    def _goal_response(self, future, generation):
        goal_handle = future.result()
        if generation != self.goal_generation:
            # A newer path was sent before this acceptance came back. Never
            # adopt the handle, but make sure the controller drops the goal.
            if goal_handle.accepted:
                self.get_logger().info(
                    'Canceling goal that was superseded before acceptance.')
                goal_handle.cancel_goal_async()
            return
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
        result_future.add_done_callback(
            lambda f, gen=generation: self.get_result_callback(f, gen))

    def get_result_callback(self, future, generation):
        # See goal_response_callback: never let an exception escape into rclpy.
        try:
            self._goal_result(future, generation)
        except Exception as e:  # noqa: BLE001 — last-resort containment
            self.get_logger().error(f'get_result_callback failed: {e}')
            self.clear()
            self.publish_status(MissionStatus.FAILED, 'Mission execution failed',
                                f'Internal error in goal result handling: {e}')

    def _goal_result(self, future, generation):
        result = future.result()
        if generation != self.goal_generation:
            # Late result of a goal that was already replaced (Nav2 reports a
            # preempted goal as ABORTED). The replacement is executing; wiping
            # state here is what used to break the Stop button.
            self.get_logger().info(
                f'Ignoring stale result (status {result.status}) from a superseded goal.')
            return
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
            # Cancellation completion is normally handled by the after_cancel_*
            # callback of whoever requested pause/replace/abort — but that
            # callback is driven by the cancel-SERVICE response, which can be
            # delayed or lost (seen in the field over the zenoh bridge: robot
            # stopped, yet Resume stayed disabled because can_resume=True was
            # never published). The goal RESULT is the authoritative signal
            # that the controller stopped, so finalize the cancellation here
            # as well; cancel_in_progress makes both paths run-once.
            reason = self.cancel_reason or 'unspecified'
            self.get_logger().info(f'Goal canceled ({reason}).')
            if self.cancel_in_progress:
                self.cancel_in_progress = False
                self.current_speed = 0.0
                if self.pending_labeled_path_msg is not None:
                    pending = self.pending_labeled_path_msg
                    self.pending_labeled_path_msg = None
                    self.replace_queue_and_send(pending)
                elif self.paused:
                    self.publish_status(MissionStatus.PAUSED, 'Paused; mission retained')
                elif reason == 'abort':
                    self.current_item = None
                    self.publish_status(MissionStatus.ABORTED, 'Mission aborted by operator')
            elif self.current_item is not None:
                # Externally canceled (robot-side stop, another action client,
                # controller shutdown) -- no operator-side request set
                # cancel_in_progress. Keep the mission and present it as a
                # resumable pause: Resume re-sends the remaining stretch from
                # the robot position (and the panel re-arms the controllers).
                self.current_speed = 0.0
                self.paused = True
                self.get_logger().warn(
                    'Goal canceled externally; mission retained as paused.')
                self.publish_status(
                    MissionStatus.PAUSED,
                    'Execution stopped externally; press Resume to continue '
                    'from the robot position', can_resume=True)
        else:
            if self.paused and self.current_item is not None:
                # A requested pause can race the controller into ABORTED
                # instead of CANCELED; the robot is stopped and the segment is
                # retained, so this is a completed pause, not a lost mission.
                self.cancel_in_progress = False
                self.current_speed = 0.0
                self.get_logger().warn(
                    f'Goal ended with status {result.status} while a pause was '
                    'pending; treating it as paused.')
                self.publish_status(MissionStatus.PAUSED, 'Paused; mission retained')
                return
            if self.cancel_in_progress and self.pending_labeled_path_msg is not None:
                # A replace-cancel can likewise race into ABORTED instead of
                # CANCELED; the new mission is already pending, so hand over
                # to it instead of declaring the mission failed.
                self.cancel_in_progress = False
                self.current_speed = 0.0
                pending = self.pending_labeled_path_msg
                self.pending_labeled_path_msg = None
                self.replace_queue_and_send(pending)
                return
            if (result.status == GoalStatus.STATUS_ABORTED and
                    self.current_item is not None):
                # Aborted without any operator-side request: typically a
                # robot-side stop (controller deactivated, e-stop cycle,
                # MCS intervention). The mission is still sound -- keep it
                # as a resumable pause instead of a dead FAILED that forces
                # re-executing the stale route from segment one.
                self.current_speed = 0.0
                self.paused = True
                self.get_logger().warn(
                    'Goal aborted externally; mission retained as paused. '
                    'Resume retries the remaining stretch from the robot position.')
                self.publish_status(
                    MissionStatus.PAUSED,
                    'Execution aborted externally (controller stop?); press '
                    'Resume to continue from the robot position', can_resume=True)
                return
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
