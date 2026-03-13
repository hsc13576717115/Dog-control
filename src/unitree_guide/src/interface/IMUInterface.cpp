// 必须先定义M_PI的宏（避免部分编译器找不到π的定义）
#define _USE_MATH_DEFINES
#include "interface/IMUInterface.h"  
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cmath>
#include <sys/types.h>  // 补充：定义ssize_t类型（部分编译器需显式包含）

using namespace std;

// 构造函数：初始化参数
IMUInterface::IMUInterface(const string& port, int baud_rate) 
    : _port(port), _baud_rate(baud_rate), _serial_fd(-1), _is_serial_init(false),
      _linear_acc_x(0.0), _linear_acc_y(0.0), _linear_acc_z(0.0),
      _angular_vel_x(0.0), _angular_vel_y(0.0), _angular_vel_z(0.0),
      _quat_x(0.0), _quat_y(0.0), _quat_z(0.0), _quat_w(1.0) {}

// 析构函数：关闭串口
IMUInterface::~IMUInterface() {
    if (_serial_fd != -1) {
        close(_serial_fd);
        cout << "[IMU] 串口已关闭" << endl;
    }
}

// 辅助函数：波特率映射（将数值转为termios需要的宏）
speed_t IMUInterface::_baudToSpeedT(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default: return B230400; // 默认230400
    }
}

// 串口初始化
int IMUInterface::init() {
    // 1. 打开串口（O_NDELAY：非阻塞模式，避免阻塞主控制循环）
    _serial_fd = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (_serial_fd == -1) {
        cerr << "[IMU] 串口打开失败：" << _port << "（请检查权限或串口是否存在）" << endl;
        return -1;
    }

    // 2. 配置串口属性
    struct termios options;
    if (tcgetattr(_serial_fd, &options) != 0) { // 新增：检查tcgetattr是否成功
        cerr << "[IMU] 获取串口属性失败" << endl;
        close(_serial_fd);
        _serial_fd = -1;
        return -1;
    }

    // 基础配置：8位数据、无校验、1位停止位
    options.c_cflag |= CLOCAL | CREAD;  // 忽略调制解调器状态、启用接收
    options.c_cflag &= ~CSIZE;          // 清除数据位掩码
    options.c_cflag |= CS8;             // 8位数据位
    options.c_cflag &= ~PARENB;         // 无校验位
    options.c_cflag &= ~CSTOPB;         // 1位停止位
    options.c_iflag |= IGNPAR;          // 忽略帧错误/奇偶校验错
    options.c_oflag = 0;                // 原始输出模式（无转义处理）
    options.c_lflag = 0;                // 原始输入模式（无终端行处理）

    // 超时配置：非阻塞读取（0：有数据立即返回，无数据返回0）
    options.c_cc[VTIME] = 0; // 读取超时（单位：0.1s，0=无超时）
    options.c_cc[VMIN] = 0;  // 最小读取字节数（0=不等待，直接返回已读数据）

    // 3. 设置波特率
    speed_t speed = _baudToSpeedT(_baud_rate);
    cfsetospeed(&options, speed); // 设置输出波特率
    cfsetispeed(&options, speed); // 设置输入波特率

    // 4. 应用配置并清空缓存（TCSANOW：立即生效）
    tcflush(_serial_fd, TCIFLUSH); // 清空输入缓存，避免旧数据干扰
    if (tcsetattr(_serial_fd, TCSANOW, &options) != 0) {
        cerr << "[IMU] 配置串口属性失败" << endl;
        close(_serial_fd);
        _serial_fd = -1;
        return -1;
    }

    _is_serial_init = true;
    cout << "[IMU] 串口初始化成功：" << _port << "（波特率：" << _baud_rate << "bps）" << endl;
    return 0;
}

// 读取并解析IMU数据（非阻塞，避免影响主控制循环）
int IMUInterface::readAndParseData() {
    if (!_is_serial_init || _serial_fd == -1) {
        cerr << "[IMU] 串口未初始化，无法读取数据" << endl;
        return -1;
    }

    // 1. 读取帧头（0x55）：非阻塞读取，无数据直接返回
    ssize_t ret = read(_serial_fd, _buffer, 1);
    if (ret != 1 || _buffer[0] != 0x55) {
        return 0; // 无数据或帧头错误，不报错（避免日志刷屏）
    }

    // 2. 读取剩余10字节（1字节类型 + 8字节数据 + 1字节校验和）
    ret = read(_serial_fd, _buffer + 1, 10);
    if (ret != 10) {
        cerr << "[IMU] 数据读取不完整（需10字节，实际读" << ret << "字节）" << endl;
        tcflush(_serial_fd, TCIFLUSH); // 清空缓存，避免后续帧错乱
        return -1;
    }

    // 3. 校验和验证（前10字节累加和 == 第11字节）
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += _buffer[i];
    }
    if (sum != _buffer[10]) {
        cerr << "[IMU] 校验和错误（计算：" << (int)sum << "，实际：" << (int)_buffer[10] << "）" << endl;
        return -1;
    }

    // 4. 解析数据：每个case用大括号隔离（解决变量作用域问题）
    switch (_buffer[1]) {
        case 0x51: { // 加速度数据（ax, ay, az）
            memcpy(_ax, _buffer + 2, 6); // 6字节：ax(2字节) + ay(2字节) + az(2字节)
            // 转换公式：原始值/32768 * 量程(±16g) * g(9.80665m/s²)
            _linear_acc_x = _ax[0] / 32768.0 * 16.0 * 9.80665;
            _linear_acc_y = _ax[1] / 32768.0 * 16.0 * 9.80665;
            _linear_acc_z = _ax[2] / 32768.0 * 16.0 * 9.80665;
            break;
        }

        case 0x52: { // 角速度数据（gx, gy, gz）
            memcpy(_gx, _buffer + 2, 6); // 6字节：gx(2字节) + gy(2字节) + gz(2字节)
            // 转换公式：原始值/32768 * 量程(±2000°/s) * π/180（转rad/s）
            _angular_vel_x = _gx[0] / 32768.0 * 2000.0 * M_PI / 180.0;
            _angular_vel_y = _gx[1] / 32768.0 * 2000.0 * M_PI / 180.0;
            _angular_vel_z = _gx[2] / 32768.0 * 2000.0 * M_PI / 180.0;
            break;
        }

        case 0x53: { // 欧拉角数据（roll, pitch, yaw）→ 转四元数
            memcpy(_sAngle, _buffer + 2, 6); // 6字节：roll(2) + pitch(2) + yaw(2)
            // 转换：原始值/32768 * 量程(±180°) → 转rad
            double roll = _sAngle[0] / 32768.0 * 180.0 * M_PI / 180.0;
            double pitch = _sAngle[1] / 32768.0 * 180.0 * M_PI / 180.0;
            double yaw = _sAngle[2] / 32768.0 * 180.0 * M_PI / 180.0;

            // 欧拉角转四元数（roll-x轴旋转，pitch-y轴旋转，yaw-z轴旋转，右手定则）
            tf2::Quaternion quat;
            quat.setRPY(roll, pitch, yaw);
            _quat_x = quat.x();
            _quat_y = quat.y();
            _quat_z = quat.z();
            _quat_w = quat.w();
            break;
        }

        default: { // 未知帧类型
            cerr << "[IMU] 未知帧类型：0x" << hex << (int)_buffer[1] << dec << endl;
            return -1;
        }
    }

    return 0; // 解析成功
}

// -------------------------- ROS相关功能（仅COMPILE_WITH_ROS时生效）--------------------------
#ifdef COMPILE_WITH_ROS
// 初始化ROS发布器（话题名默认"imu"，队列大小100）
void IMUInterface::initROSPublisher(ros::NodeHandle& nh, const string& topic) {
    _imu_pub = nh.advertise<sensor_msgs::Imu>(topic, 100);
    _imu_msg.header.frame_id = "base_link"; // 坐标系与机器人底盘一致

    // 初始化IMU消息默认值（避免未解析时发布空值）
    _imu_msg.orientation.x = 0.0;
    _imu_msg.orientation.y = 0.0;
    _imu_msg.orientation.z = 0.0;
    _imu_msg.orientation.w = 1.0;
    _imu_msg.linear_acceleration.x = 0.0;
    _imu_msg.linear_acceleration.y = 0.0;
    _imu_msg.linear_acceleration.z = 0.0;
    _imu_msg.angular_velocity.x = 0.0;
    _imu_msg.angular_velocity.y = 0.0;
    _imu_msg.angular_velocity.z = 0.0;

    cout << "[IMU] ROS发布器初始化成功：话题名 = " << topic << "，坐标系 = base_link" << endl;
}

// 发布IMU数据到ROS（需先调用initROSPublisher初始化）
void IMUInterface::publishIMUData() {
    if (!_imu_pub) { // 检查发布器是否初始化
        cerr << "[IMU] ROS发布器未初始化，无法发布数据" << endl;
        return;
    }

    // 填充时间戳（每次发布更新当前时间）
    _imu_msg.header.stamp = ros::Time::now();

    // 填充解析后的IMU数据
    _imu_msg.linear_acceleration.x = _linear_acc_x;
    _imu_msg.linear_acceleration.y = _linear_acc_y;
    _imu_msg.linear_acceleration.z = _linear_acc_z;
    _imu_msg.angular_velocity.x = _angular_vel_x;
    _imu_msg.angular_velocity.y = _angular_vel_y;
    _imu_msg.angular_velocity.z = _angular_vel_z;
    _imu_msg.orientation.x = _quat_x;
    _imu_msg.orientation.y = _quat_y;
    _imu_msg.orientation.z = _quat_z;
    _imu_msg.orientation.w = _quat_w;

    // 发布消息
    _imu_pub.publish(_imu_msg);
}
#endif