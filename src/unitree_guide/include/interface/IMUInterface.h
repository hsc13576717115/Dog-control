#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <string>
#include <cstdint>
#include <tf2/LinearMath/Quaternion.h>

#include <termios.h>  
#ifdef COMPILE_WITH_ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#endif

class IMUInterface {
public:
    // 构造函数：传入串口参数、ROS节点句柄（可选）
    IMUInterface(const std::string& port = "/dev/ttyS6", int baud_rate = 230400);
    ~IMUInterface(); // 析构函数：关闭串口

    // 初始化串口（返回0成功，-1失败）
    int init();

    // 读取并解析IMU数据（返回0成功，-1失败）
    int readAndParseData();

    // ROS发布IMU数据（仅在COMPILE_WITH_ROS时生效）
    #ifdef COMPILE_WITH_ROS
    void initROSPublisher(ros::NodeHandle& nh, const std::string& topic = "imu");
    void publishIMUData();
    #endif

    // 获取解析后的IMU数据（供外部调用，如控制算法）
    double getLinearAccX() const { return _linear_acc_x; }
    double getLinearAccY() const { return _linear_acc_y; }
    double getLinearAccZ() const { return _linear_acc_z; }
    double getAngularVelX() const { return _angular_vel_x; }
    double getAngularVelY() const { return _angular_vel_y; }
    double getAngularVelZ() const { return _angular_vel_z; }
    double getQuatX() const { return _quat_x; }
    double getQuatY() const { return _quat_y; }
    double getQuatZ() const { return _quat_z; }
    double getQuatW() const { return _quat_w; }

private:
    // 串口参数
    std::string _port;       // 串口号（如/dev/ttyS6）
    int _baud_rate;          // 波特率（230400）
    int _serial_fd;          // 串口文件描述符
    bool _is_serial_init;    // 串口是否初始化成功

    // IMU原始数据缓冲区
    uint8_t _buffer[11];     // 帧长度：1（0x55）+1（类型）+8（数据）+1（校验和）=11字节
    int16_t _ax[3];          // 原始加速度（x/y/z）
    int16_t _gx[3];          // 原始角速度（x/y/z）
    int16_t _sAngle[3];      // 原始欧拉角（x/y/z）

    // 解析后的物理量（国际单位）
    double _linear_acc_x;    // 线加速度 x (m/s²)
    double _linear_acc_y;    // 线加速度 y (m/s²)
    double _linear_acc_z;    // 线加速度 z (m/s²)
    double _angular_vel_x;   // 角速度 x (rad/s)
    double _angular_vel_y;   // 角速度 y (rad/s)
    double _angular_vel_z;   // 角速度 z (rad/s)
    double _quat_x;          // 四元数 x
    double _quat_y;          // 四元数 y
    double _quat_z;          // 四元数 z
    double _quat_w;          // 四元数 w

    // ROS发布器（仅ROS模式）
    #ifdef COMPILE_WITH_ROS
    ros::Publisher _imu_pub;
    sensor_msgs::Imu _imu_msg;
    #endif

    // 辅助函数：波特率转换（系统需要B230400等宏，这里做映射）
    speed_t _baudToSpeedT(int baud);
};

#endif // IMU_INTERFACE_H