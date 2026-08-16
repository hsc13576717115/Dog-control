#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


HELP = """
Custom Dog keyboard control (unitree_guide-compatible layout)

  1: PASSIVE        2: calibrate + stand up + arm locomotion
  Zero velocity automatically holds MPC_STANCE; motion automatically starts TROT.
  ESC: emergency stop

  W/S: increase/decrease forward speed
  A/D: increase left/right speed
  J/L: increase left/right yaw rate
  SPACE or X: zero all velocity commands

Velocity commands are incremental and remain active until changed or zeroed.
Press Ctrl-C to exit.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('publish_rate', 20.0)
        # Match the controller's validated command envelope. A small
        # normalized increment keeps terminal key-repeat controllable while
        # still allowing the complete range to be reached deliberately.
        self.declare_parameter('increment', 0.05)
        self.declare_parameter('max_speed_x', 1.5)
        self.declare_parameter('max_speed_y', 1.0)
        self.declare_parameter('max_speed_yaw', 2.0)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.increment = float(self.get_parameter('increment').value)
        self.max_speed_x = float(self.get_parameter('max_speed_x').value)
        self.max_speed_y = float(self.get_parameter('max_speed_y').value)
        self.max_speed_yaw = float(self.get_parameter('max_speed_yaw').value)
        if self.publish_rate <= 0.0 or self.increment <= 0.0:
            raise ValueError('publish_rate and increment must be positive')
        if min(self.max_speed_x, self.max_speed_y, self.max_speed_yaw) <= 0.0:
            raise ValueError('velocity limits must be positive')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.joy_publisher = self.create_publisher(Joy, '/joy', 1)
        self.command = Twist()
        self.forward = 0.0
        self.lateral = 0.0
        self.yaw = 0.0

    def publish_mode(self, button_index):
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = [0.0] * 6
        message.buttons = [0] * 10
        message.buttons[button_index] = 1
        self.joy_publisher.publish(message)

    def handle_key(self, key):
        mode_buttons = {
            '1': 1,       # Unitree L2+B: PASSIVE / acknowledge FAULT
            '2': 7,       # Unitree L2+A: calibrate and stand up
            '\x1b': 3,    # Software emergency stop
        }
        if key in mode_buttons:
            self.publish_mode(mode_buttons[key])
            return

        if key == 'w':
            self.forward = self.clamp_normalized(self.forward + self.increment)
        elif key == 's':
            self.forward = self.clamp_normalized(self.forward - self.increment)
        elif key == 'a':
            self.lateral = self.clamp_normalized(self.lateral + self.increment)
        elif key == 'd':
            self.lateral = self.clamp_normalized(self.lateral - self.increment)
        elif key in ('j', 'q'):
            self.yaw = self.clamp_normalized(self.yaw + self.increment)
        elif key in ('l', 'e'):
            self.yaw = self.clamp_normalized(self.yaw - self.increment)
        elif key in (' ', 'x'):
            self.forward = 0.0
            self.lateral = 0.0
            self.yaw = 0.0
        else:
            return

        self.update_command()
        print(
            f'\rvx={self.command.linear.x:+.3f} m/s  '
            f'vy={self.command.linear.y:+.3f} m/s  '
            f'yaw={self.command.angular.z:+.3f} rad/s',
            end='',
            flush=True,
        )

    @staticmethod
    def clamp_normalized(value):
        return max(-1.0, min(1.0, value))

    def update_command(self):
        self.command.linear.x = self.forward * self.max_speed_x
        self.command.linear.y = self.lateral * self.max_speed_y
        self.command.angular.z = self.yaw * self.max_speed_yaw

    def publish_velocity(self):
        self.cmd_vel_publisher.publish(self.command)


def open_keyboard_input():
    if sys.stdin.isatty():
        return sys.stdin, False

    try:
        # ROS 2 Launch redirects node stdin. The controlling terminal remains
        # available and lets this node share the launch terminal safely.
        return open('/dev/tty', 'r', encoding='utf-8'), True
    except OSError as error:
        raise RuntimeError(
            'keyboard_teleop requires an interactive terminal; launch with '
            'start_keyboard:=false in headless environments'
        ) from error


def read_key(input_stream, timeout):
    readable, _, _ = select.select([input_stream], [], [], timeout)
    return input_stream.read(1) if readable else None


def main():
    input_stream, close_input = open_keyboard_input()
    rclpy.init()
    node = KeyboardTeleop()
    terminal_settings = termios.tcgetattr(input_stream)
    print(HELP, flush=True)
    try:
        tty.setcbreak(input_stream.fileno())
        period = 1.0 / node.publish_rate
        while rclpy.ok():
            key = read_key(input_stream, period)
            if key == '\x03':
                break
            if key is not None:
                node.handle_key(key.lower())
            node.publish_velocity()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(input_stream, termios.TCSADRAIN, terminal_settings)
        if close_input:
            input_stream.close()
        if rclpy.ok():
            try:
                node.command = Twist()
                node.cmd_vel_publisher.publish(node.command)
            except rclpy.exceptions.RCLError:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
