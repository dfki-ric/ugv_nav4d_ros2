#!/usr/bin/env python3
"""Bench test: does the ugv_nav4d footprint adapt to robot posture changes?

Publishes FAKE wheel/tool TF frames (two postures), watches the measured
wheelbase and footprint polygon follow, presses the update_footprint service
and verifies the planner parameters (robotSizeX/Y, footprintOffsetX) adopt
the measurement. Prints PASS/FAIL per step; exit code 0 = all passed.

Usage (planner must be running, e.g. ugv_nav4d.launch.py):
    ros2 run ugv_nav4d_ros2 test_footprint_adaptation.py

WARNING: broadcasts TF for arter/wheel_*_link and arter/tool_link. Run it
against a bench planner WITHOUT robot_state_publisher / a real robot on the
same domain, otherwise the frames conflict.
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient

from geometry_msgs.msg import PolygonStamped, TransformStamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

PLANNER = '/ugv_nav4d_ros2'
BASE = 'arter/base_link'

# posture name -> {frame: (x, y)}
POSTURES = {
    'short': {
        'arter/wheel_fl_link': (2.43, 1.5), 'arter/wheel_fr_link': (2.43, -1.5),
        'arter/wheel_rl_link': (-2.43, 1.5), 'arter/wheel_rr_link': (-2.43, -1.5),
        'arter/tool_link': (4.94, 0.0),
    },
    'extended': {
        'arter/wheel_fl_link': (3.0, 1.5), 'arter/wheel_fr_link': (3.0, -1.5),
        'arter/wheel_rl_link': (-3.0, 1.5), 'arter/wheel_rr_link': (-3.0, -1.5),
        'arter/tool_link': (6.5, 0.0),
    },
}

TOL = 0.05  # [m] comparison tolerance


class FootprintTester(Node):
    def __init__(self):
        super().__init__('footprint_adaptation_tester')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.posture = POSTURES['short']
        self.create_timer(0.05, self.broadcast_posture)

        self.wheelbase = None
        self.polygon = None
        self.create_subscription(Float32, PLANNER + '/wheelbase', self.wheelbase_cb, 10)
        self.create_subscription(PolygonStamped, PLANNER + '/footprint_real', self.polygon_cb, 10)
        self.update_client = self.create_client(Trigger, PLANNER + '/update_footprint')
        self.param_client = AsyncParameterClient(self, PLANNER.lstrip('/'))

        self.failures = []

    def broadcast_posture(self):
        now = self.get_clock().now().to_msg()
        for frame, (x, y) in self.posture.items():
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = BASE
            tf.child_frame_id = frame
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

    def wheelbase_cb(self, msg):
        self.wheelbase = float(msg.data)

    def polygon_cb(self, msg):
        xs = [p.x for p in msg.polygon.points]
        ys = [p.y for p in msg.polygon.points]
        self.polygon = (min(xs), max(xs), max(abs(y) for y in ys))

    # -- helpers ----------------------------------------------------------
    def spin_until(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self.failures.append(f'timeout waiting for {what}')
        return False

    def check(self, name, actual, expected, tol=TOL):
        ok = actual is not None and abs(actual - expected) <= tol
        print(f'  [{"PASS" if ok else "FAIL"}] {name}: {actual if actual is not None else "n/a"}'
              f' (expected {expected:.2f} +/- {tol})')
        if not ok:
            self.failures.append(name)
        return ok

    def get_params(self, names):
        future = self.param_client.get_parameters(names)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.failures.append('get_parameters timed out')
            return None
        return [v.double_value for v in future.result().values]

    def call_update(self):
        future = self.update_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        result = future.result()
        if result is None or not result.success:
            self.failures.append('update_footprint failed: '
                                 + (result.message if result else 'timeout'))
            return False
        print(f'  service: {result.message}')
        return True

    def expected_for(self, posture, margins):
        front, rear, side = margins
        xs = [x for x, _ in posture.values()]
        ys = [abs(y) for _, y in posture.values()]
        wheel_xs = [x for f, (x, _) in posture.items() if 'wheel' in f]
        min_x, max_x = min(xs) - rear, max(xs) + front
        return {
            'wheelbase': max(wheel_xs) - min(wheel_xs),
            'poly_min_x': min_x,
            'poly_max_x': max_x,
            'poly_abs_y': max(ys) + side,
            'size_x': max_x - min_x,
            'offset_x': (max_x + min_x) / 2.0,
            'size_y': 2.0 * (max(ys) + side),
        }

    def run_posture(self, name, margins, press_button):
        posture = POSTURES[name]
        expected = self.expected_for(posture, margins)
        print(f'\n=== posture "{name}" (expect wheelbase {expected["wheelbase"]:.2f} m)')
        self.posture = posture
        self.wheelbase = None
        self.polygon = None
        if not self.spin_until(
                lambda: self.wheelbase is not None and
                abs(self.wheelbase - expected['wheelbase']) <= TOL and
                self.polygon is not None and
                abs(self.polygon[1] - expected['poly_max_x']) <= TOL,
                15.0, f'measurement to settle on posture "{name}"'):
            return
        self.check('wheelbase', self.wheelbase, expected['wheelbase'])
        self.check('polygon rear x', self.polygon[0], expected['poly_min_x'])
        self.check('polygon front x', self.polygon[1], expected['poly_max_x'])
        self.check('polygon |y|', self.polygon[2], expected['poly_abs_y'])

        if press_button:
            print('  pressing update_footprint...')
            if self.call_update():
                params = self.get_params(['robotSizeX', 'robotSizeY', 'footprintOffsetX'])
                if params:
                    self.check('robotSizeX after update', params[0], expected['size_x'])
                    self.check('robotSizeY after update', params[1], expected['size_y'])
                    self.check('footprintOffsetX after update', params[2], expected['offset_x'])

    def run(self):
        print('waiting for planner...')
        if not self.spin_until(lambda: self.update_client.service_is_ready(), 15.0,
                               'update_footprint service'):
            return
        if not self.param_client.wait_for_services(timeout_sec=10.0):
            self.failures.append('parameter services unavailable')
            return
        params = self.get_params(['footprint_margin_front', 'footprint_margin_rear',
                                  'footprint_margin_y'])
        if not params:
            return
        print(f'margins front/rear/y: {params[0]} / {params[1]} / {params[2]}')

        # 1: baseline posture, live measurement + button adoption
        self.run_posture('short', params, press_button=True)
        # 2: change posture, measurement must follow WITHOUT any service call
        self.run_posture('extended', params, press_button=True)
        # 3: back to baseline, prove it shrinks again too
        self.run_posture('short', params, press_button=False)


def main():
    rclpy.init()
    tester = FootprintTester()
    try:
        tester.run()
    finally:
        failures = tester.failures
        print('\n=== RESULT:', 'ALL PASSED' if not failures
              else f'{len(failures)} FAILURE(S): ' + '; '.join(failures))
        tester.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if not failures else 1)


if __name__ == '__main__':
    main()
