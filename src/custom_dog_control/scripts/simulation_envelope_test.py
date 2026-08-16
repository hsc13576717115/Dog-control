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
    parser.add_argument('--stance-hold', type=float, default=10.0)
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
    if args.stance_hold < 0.0:
        print('FAIL: stance hold duration cannot be negative', file=sys.stderr)
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
        stance_samples = []
        next_stance_print = time.monotonic()
        while rclpy.ok() and time.monotonic() < settle_deadline:
            node.publish_command((0.0, 0.0, 0.0))
            rclpy.spin_once(node, timeout_sec=0.04)
            if node.mode != 'MPC_STANCE':
                print(f'FAIL: stance settling ended in {node.mode}', file=sys.stderr)
                return 1
            if time.monotonic() > settle_deadline - 1.0:
                stance_sample = (
                    node.value('base_vx_mps'),
                    node.value('base_vy_mps'),
                    node.angular_velocity_z,
                    node.value('base_roll_rad'),
                    node.value('base_pitch_rad'),
                    node.value('base_z_m'),
                )
                if finite(*stance_sample):
                    stance_samples.append(stance_sample)
            if time.monotonic() >= next_stance_print:
                print(
                    'stance '
                    f'v=({node.value("base_vx_mps"):+.3f},'
                    f'{node.value("base_vy_mps"):+.3f},'
                    f'{node.angular_velocity_z:+.3f}) '
                    f'z={node.value("base_z_m"):.3f} '
                    f'rp=({node.value("base_roll_rad"):+.3f},'
                    f'{node.value("base_pitch_rad"):+.3f}) '
                    f'fr_q=({node.value("fr_hip_position_rad"):+.3f},'
                    f'{node.value("fr_thigh_position_rad"):+.3f},'
                    f'{node.value("fr_calf_position_rad"):+.3f}) '
                    f'q_ref=({node.value("wbc_q_fr_hip_rad"):+.3f},'
                    f'{node.value("wbc_q_fr_thigh_rad"):+.3f},'
                    f'{node.value("wbc_q_fr_calf_rad"):+.3f}) '
                    f'qd_ref=({node.value("wbc_qd_fr_hip_rad_s"):+.3f},'
                    f'{node.value("wbc_qd_fr_thigh_rad_s"):+.3f},'
                    f'{node.value("wbc_qd_fr_calf_rad_s"):+.3f}) '
                    f'fr_qd=({node.value("fr_hip_velocity_rad_s"):+.3f},'
                    f'{node.value("fr_thigh_velocity_rad_s"):+.3f},'
                    f'{node.value("fr_calf_velocity_rad_s"):+.3f}) '
                    f'tau=({node.value("wbc_tau_fr_hip"):+.2f},'
                    f'{node.value("wbc_tau_fr_thigh"):+.2f},'
                    f'{node.value("wbc_tau_fr_calf"):+.2f}) '
                    f'fz=({node.value("policy_fz_fr"):+.1f},'
                    f'{node.value("policy_fz_fl"):+.1f},'
                    f'{node.value("policy_fz_rr"):+.1f},'
                    f'{node.value("policy_fz_rl"):+.1f})',
                    flush=True,
                )
                next_stance_print += 0.20

        if not stance_samples:
            print('FAIL: no finite pre-command stance samples', file=sys.stderr)
            return 1
        pre_stance_peaks = tuple(
            max(abs(sample[index]) for sample in stance_samples)
            for index in range(5)
        )
        pre_stance_z_error = max(
            abs(sample[5] - 0.29) for sample in stance_samples
        )
        if (
            pre_stance_peaks[0] > 0.10
            or pre_stance_peaks[1] > 0.10
            or pre_stance_peaks[2] > 0.15
            or pre_stance_peaks[3] > 0.15
            or pre_stance_peaks[4] > 0.15
            or pre_stance_z_error > 0.08
        ):
            print(
                'FAIL: unstable before command '
                f'v_peak=({pre_stance_peaks[0]:.3f},'
                f'{pre_stance_peaks[1]:.3f},'
                f'{pre_stance_peaks[2]:.3f}) '
                f'rp_peak=({pre_stance_peaks[3]:.3f},'
                f'{pre_stance_peaks[4]:.3f}) '
                f'z_error={pre_stance_z_error:.3f}',
                file=sys.stderr,
            )
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
                    f'ypr=({node.value("base_yaw_rad"):+.3f},'
                    f'{pitch:+.3f},{roll:+.3f}) '
                    f'cmd=({node.value("requested_vx_mps"):+.3f},'
                    f'{node.value("requested_vy_mps"):+.3f},'
                    f'{node.value("requested_yaw_rad_s"):+.3f}) '
                    f'phase={node.latest.get("policy_mode", "?")} '
                    f'q=({node.value("fr_hip_position_rad"):+.2f},'
                    f'{node.value("fr_thigh_position_rad"):+.2f},'
                    f'{node.value("fr_calf_position_rad"):+.2f}) '
                    f'q_ref=({node.value("wbc_q_fr_hip_rad"):+.2f},'
                    f'{node.value("wbc_q_fr_thigh_rad"):+.2f},'
                    f'{node.value("wbc_q_fr_calf_rad"):+.2f}) '
                    f'qd_ref=({node.value("wbc_qd_fr_hip_rad_s"):+.2f},'
                    f'{node.value("wbc_qd_fr_thigh_rad_s"):+.2f},'
                    f'{node.value("wbc_qd_fr_calf_rad_s"):+.2f}) '
                    f'tau_max={node.value("wbc_tau_max_abs"):.1f} '
                    f'fz=({node.value("policy_fz_fr"):+.0f},'
                    f'{node.value("policy_fz_fl"):+.0f},'
                    f'{node.value("policy_fz_rr"):+.0f},'
                    f'{node.value("policy_fz_rl"):+.0f})',
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

        post_stance_failure = ''
        if stopped and args.stance_hold > 0.0:
            hold_started = time.monotonic()
            while rclpy.ok() and time.monotonic() - hold_started < args.stance_hold:
                node.publish_command((0.0, 0.0, 0.0))
                rclpy.spin_once(node, timeout_sec=0.04)
                if node.mode != 'MPC_STANCE':
                    post_stance_failure = (
                        f'post-stop stance entered {node.mode} after '
                        f'{time.monotonic() - hold_started:.2f} s'
                    )
                    break
                hold_values = (
                    node.value('base_vx_mps'),
                    node.value('base_vy_mps'),
                    node.angular_velocity_z,
                    node.value('base_roll_rad'),
                    node.value('base_pitch_rad'),
                    node.value('base_z_m'),
                )
                if finite(*hold_values) and (
                    abs(hold_values[0]) > 0.15
                    or abs(hold_values[1]) > 0.15
                    or abs(hold_values[2]) > 0.20
                    or abs(hold_values[3]) > 0.20
                    or abs(hold_values[4]) > 0.20
                    or abs(hold_values[5] - 0.29) > 0.08
                ):
                    post_stance_failure = (
                        'post-stop stance exceeded limits '
                        f'v=({hold_values[0]:+.3f},{hold_values[1]:+.3f},'
                        f'{hold_values[2]:+.3f}) '
                        f'rp=({hold_values[3]:+.3f},{hold_values[4]:+.3f}) '
                        f'z={hold_values[5]:.3f}'
                    )
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
        if post_stance_failure:
            failures.append(post_stance_failure)

        summary = (
            f'target=({target[0]:+.2f},{target[1]:+.2f},{target[2]:+.2f}) '
            f'steady=({actual[0]:+.3f},{actual[1]:+.3f},{actual[2]:+.3f}) '
            f'rms_rp=({rms_roll:.3f},{rms_pitch:.3f}) '
            f'max_rp=({max_roll:.3f},{max_pitch:.3f}) '
            f'max_z_error={max_abs_z_error:.3f} stop={stopped} '
            f'stance_hold={args.stance_hold:.1f}s'
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
