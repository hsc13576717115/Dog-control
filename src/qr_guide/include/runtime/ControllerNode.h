#ifndef QR_GUIDE_RUNTIME_CONTROLLERNODE_H
#define QR_GUIDE_RUNTIME_CONTROLLERNODE_H

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace qr_guide {

// 输入快照：把 IMU 和 Joy 同时打包，便于控制循环按“同一拍”读取。
struct ControllerInputSnapshot {
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::Joy joy;
};

// 唯一 ROS2 Node。
// 只负责采集外部输入，不直接参与硬件控制和状态机逻辑。
class ControllerNode : public rclcpp::Node {
public:
    ControllerNode();
    ControllerInputSnapshot snapshot() const;

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    mutable std::mutex mutex_;
    sensor_msgs::msg::Imu latest_imu_;
    sensor_msgs::msg::Joy latest_joy_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

}  // namespace qr_guide

#endif  // QR_GUIDE_RUNTIME_CONTROLLERNODE_H
