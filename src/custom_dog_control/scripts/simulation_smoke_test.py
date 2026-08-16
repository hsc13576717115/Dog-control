#!/usr/bin/env python3

import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import Joy


class SimulationSmokeTest(Node):
    def __init__(self):
        super().__init__('custom_dog_simulation_smoke_test')
        self.latest = {}
        self.mode = 'NO_DIAGNOSTICS'
        self.updated = False
        self.publisher = self.create_publisher(Joy, '/joy', 10)
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/nmpc_wbc_controller/diagnostics',
            self.on_diagnostics,
            10,
        )

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name != 'custom_dog_control/nmpc_wbc':
                continue
            self.latest = {item.key: item.value for item in status.values}
            self.mode = self.latest.get('mode', status.message)
            self.updated = True

    def publish_start(self):
        message = Joy()
        message.axes = [0.0] * 6
        message.buttons = [0] * 10
        message.buttons[7] = 1
        self.publisher.publish(message)

    def value(self, key):
        try:
            return float(self.latest.get(key, 'nan'))
        except ValueError:
            return float('nan')

    def line(self, elapsed):
        return (
            f'{elapsed:5.1f}s {self.mode:10s} '
            f'xyz=({self.value("base_x_m"):+.3f},'
            f'{self.value("base_y_m"):+.3f},'
            f'{self.value("base_z_m"):+.3f}) '
            f'r={self.value("base_roll_rad"):+.3f} '
            f'p={self.value("base_pitch_rad"):+.3f} '
            f'v=({self.value("base_vx_mps"):+.2f},'
            f'{self.value("base_vy_mps"):+.2f},'
            f'{self.value("base_vz_mps"):+.2f}) '
            f'qFR=({self.value("fr_hip_position_rad"):+.2f},'
            f'{self.value("fr_thigh_position_rad"):+.2f},'
            f'{self.value("fr_calf_position_rad"):+.2f}) '
            f'tauFR=({self.value("wbc_tau_fr_hip"):+.2f},'
            f'{self.value("wbc_tau_fr_thigh"):+.2f},'
            f'{self.value("wbc_tau_fr_calf"):+.2f}) '
            f'Fz=({self.value("policy_fz_fr"):+.1f},'
            f'{self.value("policy_fz_fl"):+.1f},'
            f'{self.value("policy_fz_rr"):+.1f},'
            f'{self.value("policy_fz_rl"):+.1f}) '
            f'wbc={self.latest.get("wbc_valid", "false")}'
        )


def main():
    rclpy.init()
    node = SimulationSmokeTest()
    try:
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.mode == 'PASSIVE' and node.publisher.get_subscription_count() > 0:
                break
        else:
            print('FAIL: controller did not become ready in PASSIVE', file=sys.stderr)
            return 2

        for _ in range(10):
            node.publish_start()
            rclpy.spin_once(node, timeout_sec=0.1)

        started = time.monotonic()
        next_print = started
        reached_stance = False
        while rclpy.ok() and time.monotonic() - started < 20.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_print:
                print(node.line(now - started), flush=True)
                next_print += 0.5
            reached_stance = reached_stance or node.mode == 'MPC_STANCE'
            if node.mode == 'FAULT':
                print('FAIL: controller entered FAULT', file=sys.stderr)
                return 1
        if not reached_stance:
            print('FAIL: MPC_STANCE was not reached', file=sys.stderr)
            return 1
        print('PASS: position stand and MPC_STANCE remained active for 20 s')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
