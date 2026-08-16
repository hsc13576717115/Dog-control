#!/usr/bin/env python3

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy


class EnvelopeTest(Node):
    def __init__(self):
        super().__init__('custom_dog_simulation_envelope_test')
        self.latest = {}
        self.mode = 'NO_DIAGNOSTICS'
        self.angular_velocity_z = float('nan')
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(
            DiagnosticArray,
            '/nmpc_wbc_controller/diagnostics',
            self.on_diagnostics,
            10,
        )
        self.create_subscription(
            Odometry, '/ground_truth/odom', self.on_ground_truth, 10
        )

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name == 'custom_dog_control/nmpc_wbc':
                self.latest = {item.key: item.value for item in status.values}
                self.mode = self.latest.get('mode', status.message)

    def on_ground_truth(self, message):
        self.angular_velocity_z = message.twist.twist.angular.z

    def value(self, key):
        try:
            return float(self.latest.get(key, 'nan'))
        except ValueError:
            return float('nan')

    def publish_command(self, command):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        self.cmd_publisher.publish(message)

    def publish_button(self, index):
        message = Joy()
        message.axes = [0.0] * 6
        message.buttons = [0] * 10
        message.buttons[index] = 1
        self.joy_publisher.publish(message)

    def wait_for_mode(self, expected, timeout, button=None):
        deadline = time.monotonic() + timeout
        next_button = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if button is not None and now >= next_button:
                self.publish_button(button)
                next_button = now + 0.10
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.mode == 'FAULT':
                return False
            if self.mode == expected:
                return True
        return False


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Validate one full-scale Gazebo velocity command.'
    )
    parser.add_argument('--vx', type=float, default=0.0)
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--steady-window', type=float, default=5.0)
    return parser.parse_args()


def finite(*values):
    return all(math.isfinite(value) for value in values)


def main():
    args = parse_arguments()
    command = (args.vx, args.vy, args.yaw)
    nonzero_axes = sum(abs(value) > 1e-9 for value in command)
    if nonzero_axes != 1:
        print('FAIL: specify exactly one non-zero velocity axis', file=sys.stderr)
        return 2
    if args.duration <= args.steady_window or args.steady_window <= 0.0:
        print('FAIL: duration must exceed the positive steady window', file=sys.stderr)
        return 2

    rclpy.init()
    node = EnvelopeTest()
    try:
        if not node.wait_for_mode('PASSIVE', 30.0):
            print('FAIL: controller did not become ready in PASSIVE', file=sys.stderr)
            return 2
        stand_started = time.monotonic()
        if not node.wait_for_mode('MPC_STANCE', 30.0, button=7):
            print(f'FAIL: stand-up ended in {node.mode}', file=sys.stderr)
            return 1
        stand_elapsed = time.monotonic() - stand_started
        print(f'MPC_STANCE ready after {stand_elapsed:.2f} s', flush=True)

        settle_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            node.publish_command((0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.04)
            if node.mode != 'MPC_STANCE':
                print(f'FAIL: stance settling ended in {node.mode}', file=sys.stderr)
                return 1

        started = time.monotonic()
        samples = deque()
        max_roll = 0.0
        max_pitch = 0.0
        max_abs_z_error = 0.0
        invalid_wbc_samples = 0
        unhealthy_mpc_samples = 0
        next_print = started
        while rclpy.ok() and time.monotonic() - started < args.duration:
            node.publish_command(command)
            rclpy.spin_once(node, timeout_sec=0.04)
            elapsed = time.monotonic() - started
            if node.mode == 'FAULT':
                reason = node.latest.get('fault_reason', 'see launch output')
                print(f'FAIL: controller entered FAULT ({reason})', file=sys.stderr)
                return 1
            if elapsed > 2.0 and node.mode != 'MPC_TROT':
                print(f'FAIL: unexpected mode {node.mode}', file=sys.stderr)
                return 1

            roll = node.value('base_roll_rad')
            pitch = node.value('base_pitch_rad')
            z = node.value('base_z_m')
            vx = node.value('base_vx_mps')
            vy = node.value('base_vy_mps')
            wz = node.angular_velocity_z
            if finite(roll, pitch, z, vx, vy, wz):
                max_roll = max(max_roll, abs(roll))
                max_pitch = max(max_pitch, abs(pitch))
                max_abs_z_error = max(max_abs_z_error, abs(z - 0.29))
                samples.append((time.monotonic(), vx, vy, wz, roll, pitch))
                cutoff = time.monotonic() - args.steady_window
                while samples and samples[0][0] < cutoff:
                    samples.popleft()
            if elapsed > 1.0:
                invalid_wbc_samples += node.latest.get('wbc_valid') != 'true'
                unhealthy_mpc_samples += (
                    node.latest.get('mpc_solver_healthy') != 'true'
                )
            if time.monotonic() >= next_print:
                print(
                    f'{elapsed:5.1f}s mode={node.mode} '
                    f'v=({vx:+.3f},{vy:+.3f},{wz:+.3f}) '
                    f'rp=({roll:+.3f},{pitch:+.3f}) '
                    f'cmd=({node.value("requested_vx_mps"):+.3f},'
                    f'{node.value("requested_vy_mps"):+.3f},'
                    f'{node.value("requested_yaw_rad_s"):+.3f})',
                    flush=True,
                )
                next_print += 1.0

        if not samples:
            print('FAIL: no finite steady-state samples', file=sys.stderr)
            return 1
        count = len(samples)
        mean_vx = sum(sample[1] for sample in samples) / count
        mean_vy = sum(sample[2] for sample in samples) / count
        mean_wz = sum(sample[3] for sample in samples) / count
        rms_roll = math.sqrt(sum(sample[4] ** 2 for sample in samples) / count)
        rms_pitch = math.sqrt(sum(sample[5] ** 2 for sample in samples) / count)

        stop_started = time.monotonic()
        stopped = False
        while rclpy.ok() and time.monotonic() - stop_started < 15.0:
            node.publish_command((0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.04)
            if node.mode == 'FAULT':
                print('FAIL: controller faulted while stopping', file=sys.stderr)
                return 1
            if (
                node.mode == 'MPC_STANCE'
                and abs(node.value('base_vx_mps')) < 0.10
                and abs(node.value('base_vy_mps')) < 0.10
                and abs(node.angular_velocity_z) < 0.15
            ):
                stopped = True
                break

        actual = (mean_vx, mean_vy, mean_wz)
        target = command
        target_axis = next(index for index, value in enumerate(target) if value)
        speed_error = abs(actual[target_axis] - target[target_axis])
        allowed_error = 0.20 if target_axis < 2 else 0.30
        failures = []
        if speed_error > allowed_error:
            failures.append(
                f'steady speed error {speed_error:.3f} exceeds '
                f'{allowed_error:.3f}'
            )
        if rms_roll > 0.15 or rms_pitch > 0.15:
            failures.append(
                f'attitude RMS too large ({rms_roll:.3f}, {rms_pitch:.3f} rad)'
            )
        if max_roll > 0.35 or max_pitch > 0.35:
            failures.append(
                f'attitude peak too large ({max_roll:.3f}, {max_pitch:.3f} rad)'
            )
        if max_abs_z_error > 0.08:
            failures.append(f'height error too large ({max_abs_z_error:.3f} m)')
        if invalid_wbc_samples:
            failures.append(f'{invalid_wbc_samples} invalid WBC samples')
        if unhealthy_mpc_samples:
            failures.append(f'{unhealthy_mpc_samples} unhealthy NMPC samples')
        if not stopped:
            failures.append(f'did not stop safely (mode={node.mode})')

        summary = (
            f'target=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) '
            f'steady=({actual[0]:+.3f},{actual[1]:+.3f},{actual[2]:+.3f}) '
            f'rms_rp=({rms_roll:.3f},{rms_pitch:.3f}) '
            f'max_rp=({max_roll:.3f},{max_pitch:.3f}) '
            f'max_z_error={max_abs_z_error:.3f} stop={stopped}'
        )
        if failures:
            print(f'FAIL: {summary}', file=sys.stderr)
            for failure in failures:
                print(f'  - {failure}', file=sys.stderr)
            return 1
        print(f'PASS: {summary}')
        return 0
    finally:
        try:
            for _ in range(5):
                node.publish_command((0.0, 0.0, 0.0))
                rclpy.spin_once(node, timeout_sec=0.02)
        except rclpy.exceptions.RCLError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
