#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy


class TerrainTest(Node):
    def __init__(self):
        super().__init__('custom_dog_simulation_terrain_test')
        self.latest = {}
        self.mode = 'NO_DIAGNOSTICS'
        self.odom = None
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(
            DiagnosticArray,
            '/nmpc_wbc_controller/diagnostics',
            self.on_diagnostics,
            10,
        )
        self.create_subscription(
            Odometry, '/ground_truth/odom', self.on_odometry, 10
        )

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name == 'custom_dog_control/nmpc_wbc':
                self.latest = {item.key: item.value for item in status.values}
                self.mode = self.latest.get('mode', status.message)

    def on_odometry(self, message):
        self.odom = message

    def publish_command(self, vx=0.0):
        message = Twist()
        message.linear.x = vx
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
            if self.mode == expected and self.odom is not None:
                return True
        return False


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Validate forward traversal through one Gazebo terrain lane.'
    )
    parser.add_argument('--name', default='terrain')
    parser.add_argument('--speed', type=float, default=0.25)
    parser.add_argument('--target-distance', type=float, default=4.2)
    parser.add_argument('--timeout', type=float, default=25.0)
    parser.add_argument('--stance-hold', type=float, default=5.0)
    parser.add_argument('--max-attitude', type=float, default=0.45)
    parser.add_argument('--max-lateral-drift', type=float, default=0.35)
    parser.add_argument('--min-base-height', type=float, default=0.12)
    return parser.parse_args()


def quaternion_roll_pitch(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch


def odometry_sample(node):
    pose = node.odom.pose.pose
    twist = node.odom.twist.twist
    roll, pitch = quaternion_roll_pitch(pose.orientation)
    return {
        'x': pose.position.x,
        'y': pose.position.y,
        'z': pose.position.z,
        'vx': twist.linear.x,
        'vy': twist.linear.y,
        'wz': twist.angular.z,
        'roll': roll,
        'pitch': pitch,
    }


def publish_zero_and_spin(node, duration):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_command()
        rclpy.spin_once(node, timeout_sec=0.04)


def wait_for_stable_stance(node, timeout=10.0, stable_duration=1.0):
    deadline = time.monotonic() + timeout
    stable_since = None
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_command()
        rclpy.spin_once(node, timeout_sec=0.04)
        if node.mode == 'FAULT' or node.odom is None:
            return False
        sample = odometry_sample(node)
        stable = (
            node.mode == 'MPC_STANCE'
            and abs(sample['vx']) < 0.05
            and abs(sample['vy']) < 0.05
            and abs(sample['wz']) < 0.10
            and abs(sample['roll']) < 0.15
            and abs(sample['pitch']) < 0.15
            and 0.22 < sample['z'] < 0.36
        )
        if stable:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= stable_duration:
                return True
        else:
            stable_since = None
    return False


def main():
    args = parse_arguments()
    if args.speed <= 0.0 or args.target_distance <= 0.0 or args.timeout <= 0.0:
        print('FAIL: speed, target distance, and timeout must be positive', file=sys.stderr)
        return 2

    rclpy.init()
    node = TerrainTest()
    failures = []
    try:
        if not node.wait_for_mode('PASSIVE', 30.0):
            print('FAIL: controller did not become ready in PASSIVE', file=sys.stderr)
            return 2
        if not node.wait_for_mode('MPC_STANCE', 35.0, button=7):
            reason = node.latest.get('fault_reason', node.mode)
            print(f'FAIL: stand-up ended in {reason}', file=sys.stderr)
            return 1

        # MPC_STANCE begins at the start of the position-to-WBC handoff. Wait
        # beyond that blend, then require a continuous quiet window so the
        # traversal metrics cannot include residual stand-up motion.
        publish_zero_and_spin(node, 3.0)
        if not wait_for_stable_stance(node):
            reason = node.latest.get('fault_reason', node.mode)
            print(
                f'FAIL: pre-traversal stance did not stabilize ({reason})',
                file=sys.stderr,
            )
            return 1

        initial = odometry_sample(node)
        started = time.monotonic()
        next_print = started
        reached_target = False
        max_roll = 0.0
        max_pitch = 0.0
        max_lateral_drift = 0.0
        min_z = initial['z']
        max_z = initial['z']
        invalid_wbc_samples = 0
        unhealthy_mpc_samples = 0

        while rclpy.ok() and time.monotonic() - started < args.timeout:
            node.publish_command(args.speed)
            rclpy.spin_once(node, timeout_sec=0.04)
            if node.mode == 'FAULT':
                failures.append(
                    'controller entered FAULT: ' +
                    node.latest.get('fault_reason', 'unknown')
                )
                break

            sample = odometry_sample(node)
            distance = sample['x'] - initial['x']
            lateral_drift = abs(sample['y'] - initial['y'])
            max_roll = max(max_roll, abs(sample['roll']))
            max_pitch = max(max_pitch, abs(sample['pitch']))
            max_lateral_drift = max(max_lateral_drift, lateral_drift)
            min_z = min(min_z, sample['z'])
            max_z = max(max_z, sample['z'])
            if time.monotonic() - started > 1.0:
                invalid_wbc_samples += node.latest.get('wbc_valid') != 'true'
                unhealthy_mpc_samples += (
                    node.latest.get('mpc_solver_healthy') != 'true'
                )

            if time.monotonic() >= next_print:
                print(
                    f'{args.name}: x={distance:+.3f} '
                    f'y={sample["y"] - initial["y"]:+.3f} '
                    f'z={sample["z"]:.3f} '
                    f'rp=({sample["roll"]:+.3f},{sample["pitch"]:+.3f}) '
                    f'v=({sample["vx"]:+.3f},{sample["vy"]:+.3f}) '
                    f'mode={node.mode}',
                    flush=True,
                )
                next_print += 1.0

            if distance >= args.target_distance:
                reached_target = True
                break

        traversal_elapsed = time.monotonic() - started
        final = odometry_sample(node)
        distance = final['x'] - initial['x']

        if not reached_target and not failures:
            failures.append(
                f'target distance not reached ({distance:.3f} < '
                f'{args.target_distance:.3f} m)'
            )
        if max_roll > args.max_attitude:
            failures.append(
                f'roll exceeded limit ({max_roll:.3f} > '
                f'{args.max_attitude:.3f} rad)'
            )
        if max_pitch > args.max_attitude:
            failures.append(
                f'pitch exceeded limit ({max_pitch:.3f} > '
                f'{args.max_attitude:.3f} rad)'
            )
        if max_lateral_drift > args.max_lateral_drift:
            failures.append(
                f'lateral drift exceeded limit ({max_lateral_drift:.3f} > '
                f'{args.max_lateral_drift:.3f} m)'
            )
        if min_z < args.min_base_height:
            failures.append(
                f'base height fell below limit ({min_z:.3f} < '
                f'{args.min_base_height:.3f} m)'
            )
        if invalid_wbc_samples:
            failures.append(f'{invalid_wbc_samples} invalid WBC samples')
        if unhealthy_mpc_samples:
            failures.append(f'{unhealthy_mpc_samples} unhealthy NMPC samples')

        if node.mode != 'FAULT':
            stop_deadline = time.monotonic() + 15.0
            stopped = False
            while rclpy.ok() and time.monotonic() < stop_deadline:
                node.publish_command()
                rclpy.spin_once(node, timeout_sec=0.04)
                sample = odometry_sample(node)
                if node.mode == 'FAULT':
                    failures.append(
                        'controller faulted while stopping: ' +
                        node.latest.get('fault_reason', 'unknown')
                    )
                    break
                if (
                    node.mode == 'MPC_STANCE'
                    and abs(sample['vx']) < 0.10
                    and abs(sample['vy']) < 0.10
                    and abs(sample['wz']) < 0.15
                ):
                    stopped = True
                    break
            if not stopped and node.mode != 'FAULT':
                failures.append('robot did not stop in MPC_STANCE')

            hold_deadline = time.monotonic() + args.stance_hold
            while (
                not failures
                and rclpy.ok()
                and time.monotonic() < hold_deadline
            ):
                node.publish_command()
                rclpy.spin_once(node, timeout_sec=0.04)
                sample = odometry_sample(node)
                roll = abs(sample['roll'])
                pitch = abs(sample['pitch'])
                if node.mode != 'MPC_STANCE':
                    failures.append(
                        f'post-traversal hold entered {node.mode}'
                    )
                elif max(roll, pitch) > args.max_attitude:
                    failures.append('post-traversal stance attitude unstable')

        print(
            'RESULT '
            f'name={args.name} distance_m={distance:.3f} '
            f'elapsed_s={traversal_elapsed:.2f} '
            f'max_roll_rad={max_roll:.3f} max_pitch_rad={max_pitch:.3f} '
            f'max_lateral_drift_m={max_lateral_drift:.3f} '
            f'z_range_m=[{min_z:.3f},{max_z:.3f}] '
            f'invalid_wbc={invalid_wbc_samples} '
            f'unhealthy_mpc={unhealthy_mpc_samples}',
            flush=True,
        )
        if failures:
            for failure in failures:
                print(f'FAIL: {failure}', file=sys.stderr)
            return 1
        print(f'PASS: {args.name} terrain traversal and stance recovery')
        return 0
    finally:
        if rclpy.ok():
            publish_zero_and_spin(node, 0.20)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
