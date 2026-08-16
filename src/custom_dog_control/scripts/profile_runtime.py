#!/usr/bin/env python3
import csv
import glob
import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node


def read_number(path, scale=1.0):
    try:
        with open(path, 'r', encoding='ascii') as stream:
            return float(stream.read().strip()) * scale
    except (OSError, ValueError):
        return float('nan')


def cpu_temperature_c():
    values = [
        read_number(path, 0.001)
        for path in glob.glob('/sys/class/thermal/thermal_zone*/temp')
    ]
    values = [value for value in values if value == value]
    return max(values) if values else float('nan')


def cpu_frequency_mhz():
    values = [
        read_number(path, 0.001)
        for path in glob.glob(
            '/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq')
    ]
    values = [value for value in values if value == value]
    return min(values) if values else float('nan')


def memory_available_mb():
    try:
        with open('/proc/meminfo', 'r', encoding='ascii') as stream:
            fields = {
                line.split(':', 1)[0]: line.split(':', 1)[1].strip()
                for line in stream
                if ':' in line
            }
        return float(fields['MemAvailable'].split()[0]) / 1024.0
    except (OSError, KeyError, ValueError):
        return float('nan')


class RuntimeProfiler(Node):
    FIELDS = [
        'elapsed_s',
        'mode',
        'control_period_mean_ms',
        'control_period_p95_ms',
        'control_period_p99_ms',
        'io_period_p99_ms',
        'mpc_solve_mean_ms',
        'mpc_solve_p95_ms',
        'mpc_solve_p99_ms',
        'io_timeout_count',
        'cpu_temperature_c',
        'minimum_cpu_frequency_mhz',
        'memory_available_mb',
    ]

    def __init__(self):
        super().__init__('custom_dog_control_runtime_profiler')
        self.declare_parameter('duration_s', 1800.0)
        self.declare_parameter('output_csv', '/tmp/custom_dog_control_profile.csv')
        self.declare_parameter('control_rate_hz', 250.0)
        self.declare_parameter('mpc_p99_limit_ms', 16.0)
        self.duration = self.get_parameter('duration_s').value
        self.output_path = self.get_parameter('output_csv').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.mpc_limit = self.get_parameter('mpc_p99_limit_ms').value
        self.started = time.monotonic()
        self.latest = {}
        self.rows = []
        self.first_timeout_count = None
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/nmpc_wbc_controller/diagnostics',
            self.on_diagnostics,
            10,
        )
        self.timer = self.create_timer(1.0, self.sample)
        self.get_logger().info(
            f'Profiling for {self.duration:.0f}s -> {self.output_path}')

    def on_diagnostics(self, message):
        for status in message.status:
            if status.name != 'custom_dog_control/nmpc_wbc':
                continue
            self.latest = {item.key: item.value for item in status.values}
            self.latest['mode'] = self.latest.get('mode', status.message)

    def metric(self, name):
        try:
            return float(self.latest.get(name, 'nan'))
        except ValueError:
            return float('nan')

    def sample(self):
        elapsed = time.monotonic() - self.started
        row = {
            'elapsed_s': elapsed,
            'mode': self.latest.get('mode', 'NO_DIAGNOSTICS'),
            'control_period_mean_ms': self.metric('control_period_mean_ms'),
            'control_period_p95_ms': self.metric('control_period_p95_ms'),
            'control_period_p99_ms': self.metric('control_period_p99_ms'),
            'io_period_p99_ms': self.metric('io_period_p99_ms'),
            'mpc_solve_mean_ms': self.metric('mpc_solve_mean_ms'),
            'mpc_solve_p95_ms': self.metric('mpc_solve_p95_ms'),
            'mpc_solve_p99_ms': self.metric('mpc_solve_p99_ms'),
            'io_timeout_count': self.metric('io_timeout_count'),
            'cpu_temperature_c': cpu_temperature_c(),
            'minimum_cpu_frequency_mhz': cpu_frequency_mhz(),
            'memory_available_mb': memory_available_mb(),
        }
        if self.first_timeout_count is None and row['io_timeout_count'] == row['io_timeout_count']:
            self.first_timeout_count = row['io_timeout_count']
        self.rows.append(row)
        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        output_dir = os.path.dirname(os.path.abspath(self.output_path))
        os.makedirs(output_dir, exist_ok=True)
        with open(self.output_path, 'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)

        latest = self.rows[-1] if self.rows else {}
        period_limit = 1200.0 / self.control_rate
        control_p99 = latest.get('control_period_p99_ms', float('inf'))
        mpc_p99 = latest.get('mpc_solve_p99_ms', float('inf'))
        timeout_count = latest.get('io_timeout_count', float('inf'))
        timeout_delta = (
            timeout_count - self.first_timeout_count
            if self.first_timeout_count is not None
            else float('inf')
        )
        passed = (
            control_p99 <= period_limit
            and mpc_p99 < self.mpc_limit
            and timeout_delta == 0.0
        )
        self.get_logger().info(
            f'Profile {"PASS" if passed else "FAIL"}: '
            f'control_p99={control_p99:.3f}ms '
            f'(limit {period_limit:.3f}), '
            f'mpc_p99={mpc_p99:.3f}ms, '
            f'new_io_timeouts={timeout_delta:.0f}')
        self.get_logger().info(f'CSV written to {self.output_path}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RuntimeProfiler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
