#!/usr/bin/env python3
"""Exercise the real FDILink executable using a pseudo-terminal, never hardware.

After sourcing ROS and the serial package, run:
  python3 test_orientation_freshness.py --driver /path/to/ahrs_driver_node
"""
import argparse
import binascii
import json
import os
import pty
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from sensor_msgs.msg import Imu


def crc8(data):
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8C if value & 1 else 0)
    return value


class Packets:
    def __init__(self):
        self.sequence = 0

    def frame(self, kind, payload, bad_crc=False, end=0xFD):
        header = bytes((0xFC, kind, len(payload), self.sequence))
        self.sequence = (self.sequence + 1) % 256
        checksum = binascii.crc_hqx(payload, 0) ^ (1 if bad_crc else 0)
        return header + bytes((crc8(header), checksum >> 8, checksum & 255)) + payload + bytes((end,))

    def imu(self):
        return self.frame(0x40, struct.pack('<12fq', .1, .2, .3, 0., 0., 9.81,
                                         0., 0., 0., 25., 101325., 25., 1))

    def ahrs(self, q=(1., 0., 0., 0.), **kwargs):
        return self.frame(0x41, struct.pack('<10fq', 0., 0., 0., 0., 0., 0., *q, 1), **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver', required=True)
    parser.add_argument('--device-type', type=int, choices=(0, 1), default=0)
    parser.add_argument('--output')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('fdilink_orientation_pty_test')
    topic = '/fdilink_orientation_test_' + str(os.getpid()) + '/imu'
    received = []
    subscription = node.create_subscription(Imu, topic, lambda m: received.append((time.monotonic(), m)), 100)
    master, slave = pty.openpty()
    packets = Packets()
    timeout = .10
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([args.driver, '--ros-args', '-p', 'serial_port_:=' + os.ttyname(slave),
                                    '-p', 'device_type_:=' + str(args.device_type), '-p', 'imu_topic:=' + topic,
                                    '-p', 'orientation_timeout_s:=' + str(timeout)],
                                   stdout=log, stderr=subprocess.STDOUT)
        def spin(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                if process.poll() is not None:
                    raise AssertionError('Driver exited unexpectedly')
                rclpy.spin_once(node, timeout_sec=min(.005, max(0., end-time.monotonic())))

        def stream(duration, attitude=None):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                if attitude is not None:
                    os.write(master, attitude())
                os.write(master, packets.imu())
                spin(.005)

        def quiet_after_timeout(attitude=None):
            stream(timeout + .12, attitude)
            start = len(received)
            stream(.12, attitude)
            assert len(received) == start, 'Fresh gyro/invalid AHRS concealed stale orientation'

        try:
            spin(.7)  # DDS discovery; the driver waits for bytes on the PTY.
            stream(.15)
            assert not received, 'IMU published before any validated AHRS orientation'
            stream(.15, packets.ahrs)
            assert len(received) >= 5, 'Valid IMU/AHRS stream did not publish'
            assert abs(received[-1][1].orientation.w - 1.) < 1e-6
            initial_count = len(received)
            # An invalid but radically different quaternion must not overwrite the cache.
            received.clear()
            stream(.03, lambda: packets.ahrs(q=(0., 1., 0., 0.), bad_crc=True))
            assert received and all(abs(m.orientation.w - 1.) < 1e-6 and abs(m.orientation.x) < 1e-6
                                    for _, m in received), 'Bad CRC poisoned valid orientation'
            quiet_after_timeout(lambda: packets.ahrs(q=(0., 1., 0., 0.), bad_crc=True))
            stream(.10, packets.ahrs)
            assert received[-1][0] > time.monotonic()-.05, 'Valid AHRS did not recover publication'
            quiet_after_timeout()  # Gyro packets alone must not keep the message alive.
            stale_count = len(received)
            stream(.05, lambda: packets.ahrs(end=0))
            assert len(received) == stale_count, 'Bad frame end refreshed the orientation'
            stream(.05, lambda: packets.ahrs(q=(float('nan'), 0., 0., 0.)))
            assert len(received) == stale_count, 'Non-finite quaternion refreshed the orientation'
            # Header + partial payload, then a gap exceeding the serial timeout.
            os.write(master, packets.ahrs()[:20])
            spin(.06)
            stream(.05)
            assert len(received) == stale_count, 'Short AHRS payload refreshed the orientation'
            stream(.10, packets.ahrs)
            assert received[-1][0] > time.monotonic()-.05, 'Parser failed to recover after short payload'
            # A partial IMU frame must not publish the remaining stale bytes.
            received.clear()
            os.write(master, packets.imu()[:-1])
            spin(.06)
            assert not received, 'Short IMU payload was published'
            result = {'passed': True, 'valid_stream_messages': initial_count,
                      'device_type': args.device_type,
                      'orientation_timeout_s': timeout,
                      'checks': ['no publication before valid AHRS', 'valid stream publishes',
                                 'bad CRC neither poisons cached quaternion nor refreshes age',
                                 'gyro-only stream expires', 'valid AHRS recovers publication',
                                 'bad frame end rejected', 'non-finite quaternion rejected',
                                 'short AHRS/IMU payload rejected', 'parser recovers after short frame'],
                      'hardware_access': 'PTY only; no real serial device'}
            if args.output:
                with open(args.output, 'w') as output:
                    json.dump(result, output, indent=2)
                    output.write('\n')
            print(json.dumps(result, indent=2))
        except Exception:
            log.seek(0)
            print(log.read())
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
