#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from evdev import InputDevice, ecodes
import select
import time  # 用于精确控制频率

class EventToJoy:
    def __init__(self):
        rospy.init_node('event_to_joy', anonymous=True, disable_signals=True)  # 禁用信号处理，减少延迟
        self.joy_pub = rospy.Publisher('/joy', Joy, queue_size=2000)  # 更大队列，防止高频丢包
        
        self.event_path = '/dev/input/event6'
        try:
            self.device = InputDevice(self.event_path)
            self.device.grab()  # 独占设备，避免其他进程抢占（关键！）
            rospy.loginfo(f"独占设备: {self.device.name}")
        except Exception as e:
            rospy.logerr(f"设备打开失败: {e}")
            exit(1)
        
        # 精简Joy消息（只保留用到的轴和按键，减少数据量）
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 6  # 假设6轴都用到，若有冗余可删减
        self.joy_msg.buttons = [0] * 10  # 同理，只保留必要按键
        
        # 映射表（保持不变，但确保只包含实际用到的键）
        self.key_map = {304:0, 305:1, 307:2, 308:3, 310:4, 311:5, 314:6, 315:7, 316:8, 317:9}
        self.axis_map = {0:(0,-32768,32767), 1:(1,-32768,32767), 2:(2,0,256), 
                         3:(3,-32768,32767), 4:(4,-32768,32767), 5:(5,0,256)}

    def normalize_axis(self, value, min_val, max_val):
        """极简归一化，减少计算量"""
        if value <= min_val:
            return -1.0
        elif value >= max_val:
            return 1.0
        return 2.0 * (value - min_val) / (max_val - min_val) - 1.0

    def run(self):
        # 用time控制精确频率（比rospy.Rate更准）
        freq = 1000  # 提升到200Hz（每5ms发布一次）
        interval = 1.0 / freq
        last_time = time.time()
        
        while not rospy.is_shutdown():
            # 非阻塞读取事件（单次读取，避免循环嵌套耗时）
            r, _, _ = select.select([self.device.fd], [], [], 0)
            if r:
                event = self.device.read_one()  # 单次读取，比read()快
                if event:
                    if event.type == ecodes.EV_KEY and event.code in self.key_map:
                        self.joy_msg.buttons[self.key_map[event.code]] = event.value
                    elif event.type == ecodes.EV_ABS and event.code in self.axis_map:
                        idx, min_v, max_v = self.axis_map[event.code]
                        self.joy_msg.axes[idx] = self.normalize_axis(event.value, min_v, max_v)
            
            # 强制发布（不带header.stamp，进一步减少开销，C++端不依赖时间戳）
            self.joy_pub.publish(self.joy_msg)
            
            # 精确控制频率，避免CPU空转
            current_time = time.time()
            sleep_time = interval - (current_time - last_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_time = current_time

if __name__ == '__main__':
    try:
        converter = EventToJoy()
        converter.run()
    except KeyboardInterrupt:
        pass
