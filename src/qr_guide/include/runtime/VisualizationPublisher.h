#ifndef QR_GUIDE_RUNTIME_VISUALIZATIONPUBLISHER_H
#define QR_GUIDE_RUNTIME_VISUALIZATIONPUBLISHER_H

#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "control/CtrlComponents.h"

namespace qr_guide {

// RViz 可视化发布器。
// 负责把估计位姿、机身轨迹、整狗骨架、四足轨迹和足端跟踪统一发成 ROS2 topic。
class VisualizationPublisher {
public:
    VisualizationPublisher(const std::shared_ptr<rclcpp::Node>& node,
                           const RobotParameters& parameters);

    void publish(const ControllerContext& context, const std::string& state_label);

private:
    static constexpr int kFootTrailMaxSize = 180;
    static constexpr int kBodyPathMaxSize = 600;

    void publishOdometry(const rclcpp::Time& stamp,
                         const Vec3& position,
                         const Vec3& velocity,
                         const Vec3& angular_velocity_body,
                         const RotMat& body_to_world,
                         const geometry_msgs::msg::Quaternion& orientation) const;
    void publishTf(const rclcpp::Time& stamp,
                   const Vec3& position,
                   const geometry_msgs::msg::Quaternion& orientation) const;
    void publishJointStates(const rclcpp::Time& stamp,
                            const ControllerContext& context) const;
    void publishBodyPath(const rclcpp::Time& stamp,
                         const Vec3& position,
                         const geometry_msgs::msg::Quaternion& orientation);
    void publishMarkers(const rclcpp::Time& stamp,
                        const ControllerContext& context,
                        const std::string& state_label,
                        const Vec3& position,
                        const Vec3& velocity,
                        const RotMat& body_to_world,
                        const geometry_msgs::msg::Quaternion& orientation);

    Vec34 commandJointAngles(const UserLowlevel::LowlevelCmd& low_cmd) const;
    Vec3 kneeInHip(const Vec3& q_user, int leg_id) const;
    Vec3 controllerToVizPoint(const Vec3& controller_point) const;
    Vec3 bodyPointToWorld(const Vec3& body_point,
                          const Vec3& body_position,
                          const RotMat& body_to_world) const;

    geometry_msgs::msg::Point toPoint(const Vec3& value) const;
    geometry_msgs::msg::Quaternion toQuaternion(const RotMat& rotation) const;
    void appendTrail(std::deque<geometry_msgs::msg::Point>* trail,
                     const geometry_msgs::msg::Point& point) const;
    void appendBodyPose(const geometry_msgs::msg::PoseStamped& pose);
    std::string makeStatusText(const ControllerContext& context,
                               const std::string& state_label,
                               const Vec3& position,
                               const Vec3& velocity) const;

    std::shared_ptr<rclcpp::Node> node_;
    RobotParameters parameters_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Path body_path_;
    std::array<std::deque<geometry_msgs::msg::Point>, NumLeg> actual_foot_trails_;
    std::array<std::deque<geometry_msgs::msg::Point>, NumLeg> command_foot_trails_;
    std::chrono::steady_clock::time_point last_publish_time_ =
        std::chrono::steady_clock::time_point::min();
};

}  // namespace qr_guide

#endif  // QR_GUIDE_RUNTIME_VISUALIZATIONPUBLISHER_H
