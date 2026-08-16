#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState


class CommandMatrixTest(Node):
    def __init__(self):
        super().__init__('custom_dog_command_matrix_test')
        self.latest = {}
        self.mode = 'NO_DIAGNOSTICS'
        self.joint_positions = {}
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(
            DiagnosticArray,
            '/nmpc_wbc_controller/diagnostics',
            self.on_diagnostics,
            10,
        )
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10
        )

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name == 'custom_dog_control/nmpc_wbc':
                self.latest = {item.key: item.value for item in status.values}
                self.mode = self.latest.get('mode', status.message)

    def on_joint_states(self, message):
        self.joint_positions = dict(zip(message.name, message.position))

    def publish(self, command=(0.0, 0.0, 0.0), button=None):
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = command
        self.cmd_publisher.publish(twist)
        if button is not None:
            joy = Joy()
            joy.axes = [0.0] * 6
            joy.buttons = [0] * 10
            joy.buttons[button] = 1
            self.joy_publisher.publish(joy)

    def value(self, key):
        try:
            return float(self.latest.get(key, 'nan'))
        except ValueError:
            return float('nan')

    def wait_for_mode(self, expected, timeout, command=None, button=None):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if command is not None or button is not None:
                self.publish(command or (0.0, 0.0, 0.0), button)
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.mode == 'FAULT':
                return False
            if self.mode == expected:
                return True
        return False


def wrapped_delta(final, initial):
    return math.atan2(math.sin(final - initial), math.cos(final - initial))


def run_segment(node, name, command, duration):
    if not node.wait_for_mode(
        'MPC_TROT', 5.0, command=command, button=2
    ):
        raise RuntimeError(f'{name}: failed to enter MPC_TROT ({node.mode})')

    initial = (
        node.value('base_x_m'),
        node.value('base_y_m'),
        node.value('base_yaw_rad'),
    )
    max_roll = 0.0
    max_pitch = 0.0
    max_inward_hip = 0.0
    invalid_wbc_samples = 0
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish(command)
        rclpy.spin_once(node, timeout_sec=0.04)
        if node.mode != 'MPC_TROT':
            raise RuntimeError(f'{name}: unexpected mode {node.mode}')
        max_roll = max(max_roll, abs(node.value('base_roll_rad')))
        max_pitch = max(max_pitch, abs(node.value('base_pitch_rad')))
        max_inward_hip = max(
            max_inward_hip,
            node.joint_positions.get('FR_hip_joint', 0.0),
            node.joint_positions.get('RR_hip_joint', 0.0),
            -node.joint_positions.get('FL_hip_joint', 0.0),
            -node.joint_positions.get('RL_hip_joint', 0.0),
        )
        invalid_wbc_samples += node.latest.get('wbc_valid') != 'true'

    final = (
        node.value('base_x_m'),
        node.value('base_y_m'),
        node.value('base_yaw_rad'),
    )
    if not node.wait_for_mode('MPC_STANCE', 4.0, button=0):
        raise RuntimeError(f'{name}: failed to return to MPC_STANCE ({node.mode})')
    for _ in range(75):
        node.publish()
        rclpy.spin_once(node, timeout_sec=0.04)
        if node.mode != 'MPC_STANCE':
            raise RuntimeError(
                f'{name}: unstable after stopping ({node.mode})'
            )

    result = {
        'dx': final[0] - initial[0],
        'dy': final[1] - initial[1],
        'dyaw': wrapped_delta(final[2], initial[2]),
        'max_roll': max_roll,
        'max_pitch': max_pitch,
        'max_inward_hip': max_inward_hip,
        'invalid_wbc_samples': invalid_wbc_samples,
    }
    print(
        f'{name}: dx={result["dx"]:+.3f} dy={result["dy"]:+.3f} '
        f'dyaw={result["dyaw"]:+.3f} '
        f'max_rp=({max_roll:.3f},{max_pitch:.3f}) '
        f'max_inward_hip={max_inward_hip:.3f} '
        f'invalid_wbc={invalid_wbc_samples}',
        flush=True,
    )
    return result


def main():
    rclpy.init()
    node = CommandMatrixTest()
    try:
        if not node.wait_for_mode('PASSIVE', 30.0):
            print('FAIL: controller did not become ready in PASSIVE', file=sys.stderr)
            return 2
        if not node.wait_for_mode('MPC_STANCE', 30.0, button=7):
            print(f'FAIL: stand-up ended in {node.mode}', file=sys.stderr)
            return 1

        results = {
            'forward': run_segment(node, 'forward', (0.05, 0.0, 0.0), 8.0),
            'lateral': run_segment(node, 'lateral', (0.0, 0.04, 0.0), 8.0),
            'yaw': run_segment(node, 'yaw', (0.0, 0.0, 0.18), 8.0),
        }
        failures = []
        if results['forward']['dx'] < 0.25:
            failures.append('forward displacement is too small')
        if abs(results['forward']['dy']) > 0.25:
            failures.append('forward command has excessive lateral drift')
        if results['lateral']['dy'] < 0.15:
            failures.append('lateral displacement is too small')
        if abs(results['lateral']['dx']) > 0.25:
            failures.append('lateral command has excessive forward drift')
        if results['yaw']['dyaw'] < 0.45:
            failures.append('yaw displacement is too small')
        if math.hypot(results['yaw']['dx'], results['yaw']['dy']) > 0.30:
            failures.append('yaw command has excessive translation')
        for name, result in results.items():
            if max(result['max_roll'], result['max_pitch']) > 0.35:
                failures.append(f'{name} body attitude exceeded 0.35 rad')
            if result['max_inward_hip'] > 0.35:
                failures.append(f'{name} hip adduction exceeded 0.35 rad')
            if result['invalid_wbc_samples']:
                failures.append(f'{name} had invalid WBC samples')
        if failures:
            for failure in failures:
                print(f'FAIL: {failure}', file=sys.stderr)
            return 1
        print('PASS: forward, lateral, and yaw NMPC/WBC command matrix')
        return 0
    except RuntimeError as error:
        print(f'FAIL: {error}', file=sys.stderr)
        return 1
    finally:
        for _ in range(5):
            node.publish()
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
