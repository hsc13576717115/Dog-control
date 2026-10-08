#!/usr/bin/env python3
"""Stationary synthetic IMU for mock.launch.py; never use on the real robot."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MockImu(Node):
    def __init__(self):
        super().__init__('mock_imu')
        if self.get_namespace() == '/':
            raise RuntimeError('Synthetic IMU requires an isolated, non-root namespace')
        self.declare_parameter('frame_id', 'base')
        self.publisher = self.create_publisher(Imu, 'imu', 10)
        self.timer = self.create_timer(.004, self.publish)

    def publish(self):
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('frame_id').value
        message.orientation.w = 1.
        message.linear_acceleration.z = 9.81
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = MockImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
