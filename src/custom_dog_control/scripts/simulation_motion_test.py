#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy


class SimulationMotionTest(Node):
    def __init__(self):
        super().__init__('custom_dog_simulation_motion_test')
        self.latest = {}
        self.mode = 'NO_DIAGNOSTICS'
        self.joint_positions = {}
        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/nmpc_wbc_controller/diagnostics',
            self.on_diagnostics,
            10,
        )
        self.joint_subscription = self.create_subscription(
            JointState, '/joint_states', self.on_joint_states, 10
        )

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name != 'custom_dog_control/nmpc_wbc':
                continue
            self.latest = {item.key: item.value for item in status.values}
            self.mode = self.latest.get('mode', status.message)

    def on_joint_states(self, message):
        self.joint_positions = dict(zip(message.name, message.position))

    def publish_mode(self, button_index):
        message = Joy()
        message.axes = [0.0] * 6
        message.buttons = [0] * 10
        message.buttons[button_index] = 1
        self.joy_publisher.publish(message)

    def publish_velocity(self, vx=0.0, vy=0.0, yaw=0.0):
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = yaw
        self.cmd_vel_publisher.publish(message)

    def value(self, key):
        try:
            return float(self.latest.get(key, 'nan'))
        except ValueError:
            return float('nan')

    def wait_for_mode(
        self, expected, timeout, button_index=None, velocity=None
    ):
        deadline = time.monotonic() + timeout
        next_command = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if button_index is not None and now >= next_command:
                self.publish_mode(button_index)
                next_command = now + 0.1
            if velocity is not None:
                self.publish_velocity(*velocity)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.mode == 'FAULT':
                return False
            if self.mode == expected:
                return True
        return False


def wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def finite(*values):
    return all(math.isfinite(value) for value in values)


def main():
    rclpy.init()
    node = SimulationMotionTest()
    try:
        if not node.wait_for_mode('PASSIVE', 30.0):
            print('FAIL: controller did not become ready in PASSIVE', file=sys.stderr)
            return 2

        expected_backend = {
            'nmpc_backend': 'legged::LeggedInterface',
            'wbc_backend': 'legged::WeightedWbc',
            'legged_control_revision':
                'a7f381c0367e98e31c01336e678eef47e304d40d',
        }
        backend_errors = [
            f'{key}={node.latest.get(key, "missing")}'
            for key, expected in expected_backend.items()
            if node.latest.get(key) != expected
        ]
        if backend_errors:
            print(
                'FAIL: runtime is not using the pinned legged_control '
                f'algorithm backend ({", ".join(backend_errors)})',
                file=sys.stderr,
            )
            return 1

        print(
            'PASSIVE ready with legged_control NMPC/WBC; '
            'starting position-interpolated stand-up',
            flush=True,
        )
        if not node.wait_for_mode('MPC_STANCE', 30.0, button_index=7):
            print(f'FAIL: stand-up ended in {node.mode}', file=sys.stderr)
            return 1

        settle_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            node.publish_velocity()
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.mode == 'FAULT':
                print('FAIL: controller faulted while settling in stance', file=sys.stderr)
                return 1

        start_x = node.value('base_x_m')
        start_y = node.value('base_y_m')
        start_yaw = node.value('base_yaw_rad')
        if not finite(start_x, start_y, start_yaw):
            print('FAIL: base diagnostics are not finite', file=sys.stderr)
            return 1

        # Arming Trot at zero command must keep the physical gait in stance.
        zero_arm_started = time.monotonic()
        while rclpy.ok() and time.monotonic() - zero_arm_started < 1.0:
            node.publish_velocity()
            node.publish_mode(2)
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.mode != 'MPC_STANCE':
                print(
                    f'FAIL: zero-speed Trot arm changed mode to {node.mode}',
                    file=sys.stderr,
                )
                return 1
        zero_arm_dx = node.value('base_x_m') - start_x
        zero_arm_dy = node.value('base_y_m') - start_y
        if node.latest.get('policy_mode') != '15':
            print('FAIL: zero-speed Trot arm changed the contact plan', file=sys.stderr)
            return 1
        if math.hypot(zero_arm_dx, zero_arm_dy) > 0.03:
            print('FAIL: robot moved while Trot was armed at zero speed', file=sys.stderr)
            return 1

        if not node.wait_for_mode(
            'MPC_TROT', 5.0, velocity=(0.05, 0.0, 0.0)
        ):
            print(f'FAIL: Trot request ended in {node.mode}', file=sys.stderr)
            return 1

        duration = 12.0
        started = time.monotonic()
        next_print = started
        invalid_wbc_samples = 0
        max_abs_roll = 0.0
        max_abs_pitch = 0.0
        max_inward_hip = 0.0
        while rclpy.ok() and time.monotonic() - started < duration:
            node.publish_velocity(vx=0.05)
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed = time.monotonic() - started
            if node.mode == 'FAULT':
                print('FAIL: controller entered FAULT during Trot', file=sys.stderr)
                return 1
            if node.mode != 'MPC_TROT':
                print(f'FAIL: unexpected mode {node.mode} during Trot', file=sys.stderr)
                return 1
            roll = node.value('base_roll_rad')
            pitch = node.value('base_pitch_rad')
            if finite(roll, pitch):
                max_abs_roll = max(max_abs_roll, abs(roll))
                max_abs_pitch = max(max_abs_pitch, abs(pitch))
            hip_positions = {
                name: node.joint_positions.get(name, float('nan'))
                for name in (
                    'FR_hip_joint', 'FL_hip_joint',
                    'RR_hip_joint', 'RL_hip_joint',
                )
            }
            if finite(*hip_positions.values()):
                max_inward_hip = max(
                    max_inward_hip,
                    hip_positions['FR_hip_joint'],
                    hip_positions['RR_hip_joint'],
                    -hip_positions['FL_hip_joint'],
                    -hip_positions['RL_hip_joint'],
                )
            if elapsed > 1.0 and node.latest.get('wbc_valid') != 'true':
                invalid_wbc_samples += 1
            if time.monotonic() >= next_print:
                print(
                    f'{elapsed:4.1f}s MPC_TROT '
                    f'xy=({node.value("base_x_m"):+.3f},'
                    f'{node.value("base_y_m"):+.3f}) '
                    f'yaw={node.value("base_yaw_rad"):+.3f} '
                    f'rp=({roll:+.3f},{pitch:+.3f}) '
                    f'wbc={node.latest.get("wbc_valid", "false")}',
                    flush=True,
                )
                next_print += 1.0

        end_x = node.value('base_x_m')
        end_y = node.value('base_y_m')
        end_yaw = node.value('base_yaw_rad')
        dx = end_x - start_x
        dy = end_y - start_y
        dyaw = wrapped_angle(end_yaw - start_yaw)

        stop_deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < stop_deadline:
            node.publish_velocity()
            node.publish_mode(0)
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.mode == 'FAULT':
                print('FAIL: controller faulted while stopping', file=sys.stderr)
                return 1

        final_vx = node.value('base_vx_mps')
        final_vy = node.value('base_vy_mps')
        failures = []
        if node.mode != 'MPC_STANCE':
            failures.append(f'did not return to MPC_STANCE (mode={node.mode})')
        if invalid_wbc_samples:
            failures.append(f'WBC invalid in {invalid_wbc_samples} Trot samples')
        if dx < 0.30:
            failures.append(f'forward displacement too small ({dx:.3f} m)')
        if abs(dy) > 0.30:
            failures.append(f'lateral displacement too large ({dy:.3f} m)')
        if abs(dyaw) > 0.30:
            failures.append(f'yaw drift too large ({dyaw:.3f} rad)')
        if max_abs_roll > 0.35 or max_abs_pitch > 0.35:
            failures.append(
                f'body attitude too large (roll={max_abs_roll:.3f}, '
                f'pitch={max_abs_pitch:.3f} rad)'
            )
        if max_inward_hip > 0.35:
            failures.append(
                f'hip adduction too large ({max_inward_hip:.3f} rad)'
            )
        if not finite(final_vx, final_vy) or max(abs(final_vx), abs(final_vy)) > 0.10:
            failures.append(
                f'robot did not settle (vx={final_vx:.3f}, vy={final_vy:.3f} m/s)'
            )

        summary = (
            f'dxy=({dx:+.3f},{dy:+.3f}) m, dyaw={dyaw:+.3f} rad, '
            f'max_rp=({max_abs_roll:.3f},{max_abs_pitch:.3f}) rad, '
            f'max_inward_hip={max_inward_hip:.3f} rad, '
            f'final_v=({final_vx:+.3f},{final_vy:+.3f}) m/s'
        )
        if failures:
            print(f'FAIL: {summary}', file=sys.stderr)
            for failure in failures:
                print(f'  - {failure}', file=sys.stderr)
            return 1
        print(f'PASS: position stand-up -> NMPC/WBC Trot -> NMPC/WBC stance; {summary}')
        return 0
    finally:
        try:
            for _ in range(3):
                node.publish_velocity()
                rclpy.spin_once(node, timeout_sec=0.02)
        except rclpy.exceptions.RCLError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
