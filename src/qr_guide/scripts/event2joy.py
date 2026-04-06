#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes
import select
import time

class EventToJoy(Node):
    def __init__(self):
        super().__init__('event2joy_node')
        self.declare_parameter('event_path', '/dev/input/event6')
        self.declare_parameter('publish_hz', 1000.0)

        self.joy_pub = self.create_publisher(Joy, '/joy', 2000)
        self.event_path = self.get_parameter('event_path').get_parameter_value().string_value
        self.publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value

        try:
            self.device = InputDevice(self.event_path)
            self.device.grab()
            self.get_logger().info(f'独占设备: {self.device.name}')
        except Exception as e:
            self.get_logger().error(f'设备打开失败: {e}')
            raise

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 6
        self.joy_msg.buttons = [0] * 10

        self.key_map = {304:0, 305:1, 307:2, 308:3, 310:4, 311:5, 314:6, 315:7, 316:8, 317:9}
        self.axis_map = {0:(0,-32768,32767), 1:(1,-32768,32767), 2:(2,0,256), 
                         3:(3,-32768,32767), 4:(4,-32768,32767), 5:(5,0,256)}

    def normalize_axis(self, value, min_val, max_val):
        if value <= min_val:
            return -1.0
        elif value >= max_val:
            return 1.0
        return 2.0 * (value - min_val) / (max_val - min_val) - 1.0

    def run(self):
        interval = 1.0 / self.publish_hz
        last_time = time.time()

        while rclpy.ok():
            r, _, _ = select.select([self.device.fd], [], [], 0)
            if r:
                event = self.device.read_one()
                if event:
                    if event.type == ecodes.EV_KEY and event.code in self.key_map:
                        self.joy_msg.buttons[self.key_map[event.code]] = event.value
                    elif event.type == ecodes.EV_ABS and event.code in self.axis_map:
                        idx, min_v, max_v = self.axis_map[event.code]
                        self.joy_msg.axes[idx] = self.normalize_axis(event.value, min_v, max_v)

            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

            current_time = time.time()
            sleep_time = interval - (current_time - last_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_time = current_time

    def close_device(self):
        try:
            self.device.ungrab()
        except Exception:
            pass
        self.device.close()

if __name__ == '__main__':
    rclpy.init()
    converter = None
    try:
        converter = EventToJoy()
        converter.run()
    except KeyboardInterrupt:
        pass
    finally:
        if converter is not None:
            converter.close_device()
            converter.destroy_node()
        rclpy.shutdown()
