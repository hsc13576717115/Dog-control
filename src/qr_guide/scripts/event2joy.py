#!/usr/bin/env python3
# 这个脚本把 Linux /dev/input/event* 设备转换成 ROS 2 /joy 消息。
# 这样主控侧可以继续复用标准 Joy 接口，而不需要直接依赖 evdev 事件细节。
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes
import select
import time

class EventToJoy(Node):
    def __init__(self):
        super().__init__('event2joy_node')
        self.declare_parameter('event_path', '/dev/input/event6')
        self.declare_parameter('publish_hz', 1000.0)

        joy_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.joy_pub = self.create_publisher(Joy, '/joy', joy_qos)
        self.event_path = self.get_parameter('event_path').get_parameter_value().string_value
        self.publish_hz = self.get_parameter('publish_hz').get_parameter_value().double_value

        try:
            self.device = InputDevice(self.event_path)
            self.device.grab()
            self.get_logger().info(
                f'手柄接口已打开 path={self.event_path} device="{self.device.name}"'
            )
        except Exception as e:
            self.get_logger().error(
                f'手柄接口打开失败 path={self.event_path} error={e}'
            )
            raise

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 6
        self.joy_msg.buttons = [0] * 10

        # 这里的索引布局需要和 JoystickMapper.cpp 中的按键/轴约定保持一致。
        self.key_map = {304:0, 305:1, 307:2, 308:3, 310:4, 311:5, 314:6, 315:7, 316:8, 317:9}
        self.axis_map = {0:(0,-32768,32767), 1:(1,-32768,32767), 2:(2,0,256), 
                         3:(3,-32768,32767), 4:(4,-32768,32767), 5:(5,0,256)}

    def normalize_axis(self, value, min_val, max_val):
        # 把不同原始量程统一归一化到 [-1, 1]，便于上层控制器统一处理。
        if value <= min_val:
            return -1.0
        elif value >= max_val:
            return 1.0
        return 2.0 * (value - min_val) / (max_val - min_val) - 1.0

    def run(self):
        interval = 1.0 / self.publish_hz
        last_time = time.time()

        while rclpy.ok():
            # 非阻塞轮询 event 设备，把当前时刻的全部输入合并成一帧 Joy 再发布。
            r, _, _ = select.select([self.device.fd], [], [], 0)
            if r:
                while True:
                    event = self.device.read_one()
                    if event is None:
                        break
                    if event.type == ecodes.EV_KEY and event.code in self.key_map:
                        self.joy_msg.buttons[self.key_map[event.code]] = event.value
                    elif event.type == ecodes.EV_ABS and event.code in self.axis_map:
                        idx, min_v, max_v = self.axis_map[event.code]
                        self.joy_msg.axes[idx] = self.normalize_axis(event.value, min_v, max_v)

            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_pub.publish(self.joy_msg)

            # 手工控速，避免因为 while 循环太快而占满 CPU。
            current_time = time.time()
            sleep_time = interval - (current_time - last_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_time = current_time

    def close_device(self):
        # grab 过的设备退出前要释放，否则可能影响系统中其他手柄读取程序。
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
