#include "runtime/ControllerNode.h"

namespace qr_guide {

// ControllerNode 是控制器与 ROS 世界的唯一直接边界。
// 这里刻意只保留“采集输入”职责，把控制逻辑全部留给 RobotRunner / FSM。
ControllerNode::ControllerNode()
    : rclcpp::Node("qr_guide_controller") {
    // 默认四元数设为单位姿态，避免 IMU 还没到时出现非法旋转矩阵。
    latest_imu_.orientation.w = 1.0;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::QoS(2000),
        std::bind(&ControllerNode::imuCallback, this, std::placeholders::_1));
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&ControllerNode::joyCallback, this, std::placeholders::_1));
}

ControllerInputSnapshot ControllerNode::snapshot() const {
    // 控制循环统一通过快照读取，避免中途被回调线程改写。
    std::lock_guard<std::mutex> lock(mutex_);
    return ControllerInputSnapshot{latest_imu_, latest_joy_};
}

void ControllerNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // IMU 不做额外处理，保证控制链拿到的是最新原始 ROS 消息。
    std::lock_guard<std::mutex> lock(mutex_);
    latest_imu_ = *msg;
}

void ControllerNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Joy 同样只做缓存，按键解释统一交给 JoystickMapper。
    std::lock_guard<std::mutex> lock(mutex_);
    latest_joy_ = *msg;
}

}  // namespace qr_guide
