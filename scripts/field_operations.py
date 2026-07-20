#!/usr/bin/env python3
"""Health aggregation and conservative field-safety policies for ugv_nav4d."""

import glob
import json
import math
import os
import shutil

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Float32
from std_srvs.srv import Trigger
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.action import FollowPath
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from ugv_nav4d_ros2.msg import (
    MissionStatus,
    OperationalZoneArray,
    RouteRisk,
    SystemHealth,
    TravMap,
)


class FieldOperations(Node):
    def __init__(self):
        super().__init__('ugv_nav4d_field_operations')
        self.declare_parameter('localization_timeout', 2.0)
        self.declare_parameter('map_timeout', 10.0)
        # Static/pre-generated MLS maps remain valid after their first
        # publication. Enable this only when the deployment contract requires
        # the traversability map to be refreshed periodically.
        self.declare_parameter('monitor_map_freshness', False)
        self.declare_parameter('communications_timeout', 5.0)
        self.declare_parameter('low_battery_percentage', 0.20)
        self.declare_parameter('critical_battery_percentage', 0.10)
        self.declare_parameter('low_battery_policy', 'warn')  # warn, pause, return
        self.declare_parameter('lost_communications_policy', 'warn')  # warn, pause, return, continue
        self.declare_parameter('heartbeat_required', False)
        self.declare_parameter('cpu_load_warning', 90.0)
        self.declare_parameter('cpu_temperature_warning', 85.0)
        self.declare_parameter('disk_free_warning_gb', 2.0)
        self.declare_parameter('mission_audit_file', 'ugv_nav4d_mission_audit.jsonl')

        self.health_pub = self.create_publisher(
            SystemHealth, '/ugv_nav4d_ros2/system_health', 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery_cb, 10)
        self.create_subscription(PoseStamped, '/ugv_nav4d_ros2/robot_pose', self.pose_cb, 10)
        self.create_subscription(TravMap, '/ugv_nav4d_ros2/trav_map', self.map_cb, 10)
        self.create_subscription(Empty, '/operator_heartbeat', self.heartbeat_cb, 10)
        self.create_subscription(
            MissionStatus, '/ugv_nav4d_ros2/execution_status', self.execution_cb, 10)
        self.create_subscription(
            MissionStatus, '/ugv_nav4d_ros2/planner_status', self.planner_status_cb, 10)
        self.create_subscription(
            RouteRisk, '/ugv_nav4d_ros2/route_risk', self.route_risk_cb, 10)
        self.create_subscription(
            OperationalZoneArray, '/ugv_nav4d_ros2/operational_zones', self.zones_cb, 10)
        self.create_subscription(
            Float32, '/ugv_nav4d_ros2/zone_speed_limit', self.zone_speed_limit_cb, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics_cb, 10)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        self.follow_path_client = ActionClient(self, FollowPath, '/follow_path')

        self.pause_client = self.create_client(
            Trigger, '/ugv_nav4d_ros2/pause_execution')
        self.return_client = self.create_client(
            Trigger, '/ugv_nav4d_ros2/plan_return_current')

        now = self.get_clock().now()
        self.last_pose = None
        self.last_map = None
        self.last_heartbeat = now
        self.last_execution = None
        self.last_planner_status = None
        self.last_route_risk = None
        self.last_zone_count = 0
        self.battery = None
        self.last_zone_speed_limit = 0.0
        self.diagnostic_errors = []
        self.last_audited_state = None
        self.policy_latches = set()
        self.create_timer(1.0, self.tick)

    def pose_cb(self, msg):
        del msg
        self.last_pose = self.get_clock().now()

    def map_cb(self, msg):
        del msg
        self.last_map = self.get_clock().now()

    def heartbeat_cb(self, msg):
        del msg
        self.last_heartbeat = self.get_clock().now()
        self.policy_latches.discard('communications')

    def execution_cb(self, msg):
        self.last_execution = msg
        audit_key = (msg.state, msg.current_segment, msg.summary, msg.failure_reason)
        if audit_key != self.last_audited_state:
            self.last_audited_state = audit_key
            self.write_audit('execution', {
                'state': msg.state_name,
                'summary': msg.summary,
                'failure_reason': msg.failure_reason,
                'current_segment': msg.current_segment,
                'total_segments': msg.total_segments,
                'distance_remaining': msg.distance_remaining,
            })

    def planner_status_cb(self, msg):
        self.last_planner_status = msg

    def route_risk_cb(self, msg):
        self.last_route_risk = msg

    def zones_cb(self, msg):
        self.last_zone_count = len(msg.zones)

    def write_audit(self, event, data):
        record = {
            'stamp_ns': self.get_clock().now().nanoseconds,
            'event': event,
            **data,
        }
        try:
            with open(str(self.get_parameter('mission_audit_file').value),
                      'a', encoding='utf-8') as stream:
                stream.write(json.dumps(record, sort_keys=True) + '\n')
        except OSError as exc:
            self.get_logger().error(f'Cannot write mission audit: {exc}')

    def zone_speed_limit_cb(self, limit):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        # Nav2 uses 0.0 to remove an absolute speed restriction.
        value = float(limit.data)
        if not math.isfinite(value) or value < 0.0:
            self.get_logger().error(
                f'Invalid resolved zone speed limit {value!r}; retaining the last valid value')
            value = self.last_zone_speed_limit
        else:
            self.last_zone_speed_limit = value
        msg.speed_limit = value
        self.speed_limit_pub.publish(msg)

    def battery_cb(self, msg):
        self.battery = msg
        if math.isfinite(msg.percentage) and msg.percentage > 0.0:
            low = float(self.get_parameter('low_battery_percentage').value)
            if msg.percentage > low:
                self.policy_latches.discard('battery')

    def diagnostics_cb(self, msg):
        self.diagnostic_errors = [status.name + ': ' + status.message
                                  for status in msg.status
                                  if status.level >= DiagnosticStatus.ERROR]

    @staticmethod
    def cpu_temperature():
        readings = []
        for path in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
            try:
                value = float(open(path, encoding='utf-8').read().strip())
                readings.append(value / 1000.0 if value > 1000.0 else value)
            except (OSError, ValueError):
                pass
        return max(readings) if readings else -1.0

    def age(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1.0e9

    def call_policy(self, key, policy, reason):
        if key in self.policy_latches or policy in ('warn', 'continue'):
            return
        self.policy_latches.add(key)
        if policy in ('pause', 'return') and self.pause_client.service_is_ready():
            self.get_logger().warn(f'{reason}; requesting execution pause.')
            self.pause_client.call_async(Trigger.Request())
        if policy == 'return' and self.return_client.service_is_ready():
            self.get_logger().error(f'{reason}; requesting return-path preview.')
            # Planning remains preview-gated: a human must still approve Execute.
            self.return_client.call_async(Trigger.Request())

    def add_readiness(self, msg, name, level, detail):
        msg.readiness_names.append(name)
        msg.readiness_levels.append(level)
        msg.readiness_messages.append(detail)

    def tick(self):
        pose_age = self.age(self.last_pose)
        map_age = self.age(self.last_map)
        comm_age = self.age(self.last_heartbeat)
        pose_timeout = float(self.get_parameter('localization_timeout').value)
        map_timeout = float(self.get_parameter('map_timeout').value)
        monitor_map_freshness = bool(
            self.get_parameter('monitor_map_freshness').value)
        comm_timeout = float(self.get_parameter('communications_timeout').value)
        heartbeat_required = bool(self.get_parameter('heartbeat_required').value)

        msg = SystemHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.planner_available = self.last_map is not None
        msg.controller_available = self.follow_path_client.server_is_ready()
        msg.localization_fresh = pose_age <= pose_timeout
        msg.map_fresh = (
            self.last_map is not None and
            (not monitor_map_freshness or map_age <= map_timeout))
        msg.communications_fresh = (not heartbeat_required or comm_age <= comm_timeout)
        msg.localization_age = float(pose_age if math.isfinite(pose_age) else -1.0)
        msg.map_age = float(map_age if math.isfinite(map_age) else -1.0)
        msg.communications_age = float(comm_age)
        msg.diagnostics_ok = not self.diagnostic_errors
        cpu_count = max(1, os.cpu_count() or 1)
        msg.cpu_load_percentage = float(100.0 * os.getloadavg()[0] / cpu_count)
        msg.cpu_temperature = float(self.cpu_temperature())
        msg.disk_free_gb = float(shutil.disk_usage('/').free / (1024.0 ** 3))

        if self.battery is not None:
            msg.battery_available = True
            msg.battery_percentage = float(self.battery.percentage)
            msg.battery_voltage = float(self.battery.voltage)
            if math.isfinite(self.battery.percentage) and self.battery.percentage >= 0.0:
                low = float(self.get_parameter('low_battery_percentage').value)
                critical = float(self.get_parameter('critical_battery_percentage').value)
                if self.battery.percentage <= critical:
                    msg.warnings.append('Battery is critical')
                elif self.battery.percentage <= low:
                    msg.warnings.append('Battery is below return reserve')
                if self.battery.percentage <= low:
                    self.call_policy(
                        'battery', str(self.get_parameter('low_battery_policy').value),
                        'Low battery')

        if not msg.localization_fresh:
            msg.warnings.append('Localization pose is stale or unavailable')
        if self.last_map is None:
            msg.warnings.append('Traversability map is unavailable')
        elif not msg.map_fresh:
            msg.warnings.append(
                f'Traversability map has not been refreshed for {map_age:.1f} s')
        if not msg.communications_fresh:
            msg.warnings.append('Operator heartbeat lost')
            self.call_policy(
                'communications',
                str(self.get_parameter('lost_communications_policy').value),
                'Operator communications lost')
        if self.diagnostic_errors:
            msg.warnings.extend(self.diagnostic_errors[:5])
        if msg.cpu_load_percentage >= float(self.get_parameter('cpu_load_warning').value):
            msg.warnings.append('CPU load is high')
        if (msg.cpu_temperature >= 0.0 and
                msg.cpu_temperature >= float(self.get_parameter('cpu_temperature_warning').value)):
            msg.warnings.append('CPU temperature is high')
        if msg.disk_free_gb <= float(self.get_parameter('disk_free_warning_gb').value):
            msg.warnings.append('Disk space is low')

        msg.level = SystemHealth.ERROR if (
            not msg.localization_fresh or not msg.communications_fresh or
            not msg.diagnostics_ok or not msg.controller_available
        ) else (SystemHealth.WARN if msg.warnings else SystemHealth.OK)
        route_valid = self.last_route_risk is not None and self.last_route_risk.valid
        self.add_readiness(
            msg, 'Pose',
            SystemHealth.OK if msg.localization_fresh else SystemHealth.ERROR,
            f'fresh ({pose_age:.1f} s old)' if msg.localization_fresh else 'stale or unavailable')
        self.add_readiness(
            msg, 'Traversability map',
            SystemHealth.OK if msg.map_fresh else SystemHealth.ERROR,
            'loaded' if msg.map_fresh else 'unavailable or stale')
        self.add_readiness(
            msg, 'Planner',
            SystemHealth.OK if msg.planner_available else SystemHealth.ERROR,
            self.last_planner_status.summary if self.last_planner_status else 'waiting for planner')
        self.add_readiness(
            msg, 'Route preview',
            SystemHealth.OK if route_valid else SystemHealth.WARN,
            self.last_route_risk.summary if self.last_route_risk else 'no approved preview yet')
        self.add_readiness(
            msg, 'Controller',
            SystemHealth.OK if msg.controller_available else SystemHealth.ERROR,
            '/follow_path ready' if msg.controller_available else '/follow_path action server unavailable')
        self.add_readiness(
            msg, 'Speed limit relay',
            SystemHealth.OK,
            f'active, current limit {self.last_zone_speed_limit:.2f} m/s'
            if self.last_zone_speed_limit > 0.0 else 'active, unrestricted')
        self.add_readiness(
            msg, 'Operational zones',
            SystemHealth.OK,
            f'{self.last_zone_count} active')
        self.add_readiness(
            msg, 'Diagnostics',
            SystemHealth.OK if msg.diagnostics_ok else SystemHealth.ERROR,
            'OK' if msg.diagnostics_ok else '; '.join(self.diagnostic_errors[:2]))
        self.add_readiness(
            msg, 'Battery',
            SystemHealth.OK if msg.battery_available else SystemHealth.WARN,
            f'{100.0 * msg.battery_percentage:.0f}%, {msg.battery_voltage:.1f} V'
            if msg.battery_available else 'not reported')
        self.add_readiness(
            msg, 'Operator heartbeat',
            SystemHealth.OK if msg.communications_fresh else SystemHealth.ERROR,
            'not required' if not heartbeat_required else
            ('fresh' if msg.communications_fresh else f'lost ({comm_age:.1f} s old)'))
        self.add_readiness(
            msg, 'Host resources',
            SystemHealth.WARN if (
                msg.cpu_load_percentage >= float(self.get_parameter('cpu_load_warning').value) or
                (msg.cpu_temperature >= 0.0 and msg.cpu_temperature >= float(
                    self.get_parameter('cpu_temperature_warning').value)) or
                msg.disk_free_gb <= float(self.get_parameter('disk_free_warning_gb').value)
            ) else SystemHealth.OK,
            f'CPU {msg.cpu_load_percentage:.0f}%, temp {msg.cpu_temperature:.0f} C, disk {msg.disk_free_gb:.1f} GB')
        msg.ready_to_execute = (
            msg.localization_fresh and msg.map_fresh and msg.communications_fresh and
            msg.diagnostics_ok and msg.controller_available and route_valid)
        msg.summary = ('OK' if msg.level == SystemHealth.OK else '; '.join(msg.warnings))
        self.health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FieldOperations()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
