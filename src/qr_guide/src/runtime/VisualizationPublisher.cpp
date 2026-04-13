#include "runtime/VisualizationPublisher.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common/enumClass.h"

namespace qr_guide {

namespace {

constexpr char kWorldFrameId[] = "odom";
constexpr char kBaseLinkFrameId[] = "base_link_est";
constexpr auto kVisualizationInterval = std::chrono::milliseconds(33);
constexpr auto kBodyTrailWindow = std::chrono::seconds(5);
constexpr auto kFootTrailWindow = std::chrono::seconds(5);
constexpr double kTrailDistanceThresholdM = 0.0005;
constexpr std::array<const char*, 12> kJointNames = {
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
};

const RotMat kControllerToVizReflection = [] {
    RotMat reflection = RotMat::Identity();
    reflection(1, 1) = -1.0;
    return reflection;
}();

struct Color {
    float r;
    float g;
    float b;
    float a;
};

Color LegColor(int leg_id, float alpha) {
    static constexpr Color kColors[NumLeg] = {
        {0.96f, 0.69f, 0.24f, 1.0f},  // FR
        {0.40f, 0.86f, 0.95f, 1.0f},  // FL
        {0.59f, 0.90f, 0.53f, 1.0f},  // RR
        {0.88f, 0.58f, 0.93f, 1.0f},  // RL
    };
    Color color = kColors[leg_id];
    color.a = alpha;
    return color;
}

void SetColor(visualization_msgs::msg::Marker* marker, const Color& color) {
    marker->color.r = color.r;
    marker->color.g = color.g;
    marker->color.b = color.b;
    marker->color.a = color.a;
}

bool IsFrontLeg(int leg_id) {
    return leg_id == FR || leg_id == FL;
}

bool IsLeftLeg(int leg_id) {
    return leg_id == FL || leg_id == RL;
}

}  // namespace

VisualizationPublisher::VisualizationPublisher(const std::shared_ptr<rclcpp::Node>& node,
                                               const RobotParameters& parameters)
    : node_(node),
      parameters_(parameters) {
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
        "/qr_guide/estimation/odom", rclcpp::QoS(10));
    body_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "/qr_guide/estimation/path", rclcpp::QoS(10));
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(20));
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/qr_guide/visualization/marker_array", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    body_path_.header.frame_id = kWorldFrameId;
}

void VisualizationPublisher::publish(const ControllerContext& context,
                                     const std::string& state_label) {
    if (!context.lowState || !context.lowCmd || !context.robotModel || !context.estimator) {
        return;
    }

    const auto now_steady = std::chrono::steady_clock::now();
    if (last_publish_time_ != std::chrono::steady_clock::time_point::min() &&
        now_steady - last_publish_time_ < kVisualizationInterval) {
        return;
    }
    last_publish_time_ = now_steady;

    const rclcpp::Time stamp = node_->now();
    const Vec3 position = controllerToVizPosition(context.estimator->getPosition());
    const Vec3 velocity = controllerToVizPoint(context.estimator->getVelocity());
    const RotMat body_to_world = context.lowState->getRotMat();
    const geometry_msgs::msg::Quaternion orientation = toQuaternion(body_to_world);
    const Vec3 angular_velocity_body = context.lowState->imu.getGyro();

    publishOdometry(stamp, position, velocity, angular_velocity_body, body_to_world, orientation);
    publishTf(stamp, position, orientation);
    publishJointStates(stamp, context);
    publishBodyPath(stamp, position, orientation);
    publishMarkers(stamp, context, state_label, position, velocity, body_to_world, orientation);
}

void VisualizationPublisher::resetAfterCalibration(const Vec3& controller_position) {
    controller_position_origin_ = controller_position;
    body_path_.poses.clear();
    body_path_.header.frame_id = kWorldFrameId;
    for (int leg = 0; leg < NumLeg; ++leg) {
        actual_foot_trails_[leg].clear();
    }
    last_publish_time_ = std::chrono::steady_clock::time_point::min();
}

void VisualizationPublisher::publishOdometry(const rclcpp::Time& stamp,
                                             const Vec3& position,
                                             const Vec3& velocity,
                                             const Vec3& angular_velocity_body,
                                             const RotMat& body_to_world,
                                             const geometry_msgs::msg::Quaternion& orientation) const {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = kWorldFrameId;
    odom.child_frame_id = kBaseLinkFrameId;
    odom.pose.pose.position = toPoint(position);
    odom.pose.pose.orientation = orientation;

    const Vec3 velocity_body = body_to_world.transpose() * velocity;
    odom.twist.twist.linear.x = velocity_body.x();
    odom.twist.twist.linear.y = velocity_body.y();
    odom.twist.twist.linear.z = velocity_body.z();
    odom.twist.twist.angular.x = angular_velocity_body.x();
    odom.twist.twist.angular.y = angular_velocity_body.y();
    odom.twist.twist.angular.z = angular_velocity_body.z();
    odom_pub_->publish(odom);
}

void VisualizationPublisher::publishTf(const rclcpp::Time& stamp,
                                       const Vec3& position,
                                       const geometry_msgs::msg::Quaternion& orientation) const {
    if (!tf_broadcaster_) {
        return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = kWorldFrameId;
    transform.child_frame_id = kBaseLinkFrameId;
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation = orientation;
    tf_broadcaster_->sendTransform(transform);
}

void VisualizationPublisher::publishJointStates(const rclcpp::Time& stamp,
                                                const ControllerContext& context) const {
    if (!joint_state_pub_ || !context.lowState) {
        return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name.assign(kJointNames.begin(), kJointNames.end());
    joint_state.position.resize(kJointNames.size());
    joint_state.velocity.resize(kJointNames.size());
    joint_state.effort.resize(kJointNames.size());

    for (size_t index = 0; index < kJointNames.size(); ++index) {
        const auto& motor_state = context.lowState->motorState[index];
        joint_state.position[index] = motor_state.q;
        joint_state.velocity[index] = motor_state.dq;
        joint_state.effort[index] = motor_state.tauEst;
    }

    joint_state_pub_->publish(joint_state);
}

void VisualizationPublisher::publishBodyPath(
    const rclcpp::Time& stamp,
    const Vec3& position,
    const geometry_msgs::msg::Quaternion& orientation) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = kWorldFrameId;
    pose.pose.position = toPoint(position);
    pose.pose.orientation = orientation;

    body_path_.poses.push_back(pose);
    while (!body_path_.poses.empty() &&
           (stamp - body_path_.poses.front().header.stamp).to_chrono<std::chrono::nanoseconds>() > kBodyTrailWindow) {
        body_path_.poses.erase(body_path_.poses.begin());
    }
    body_path_.header.stamp = stamp;
    body_path_pub_->publish(body_path_);
}

void VisualizationPublisher::publishMarkers(const rclcpp::Time& stamp,
                                            const ControllerContext& context,
                                            const std::string& state_label,
                                            const Vec3& position,
                                            const Vec3& velocity,
                                            const RotMat& body_to_world,
                                            const geometry_msgs::msg::Quaternion& orientation) {
    const Vec34 actual_q = context.lowState->getQ();
    const Vec34 command_q = commandJointAngles(*context.lowCmd);
    visualization_msgs::msg::MarkerArray marker_array;

    auto make_marker = [&](const std::string& ns, int id, int type) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = kWorldFrameId;
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        return marker;
    };

    auto add_line_points = [](visualization_msgs::msg::Marker* marker,
                              const geometry_msgs::msg::Point& a,
                              const geometry_msgs::msg::Point& b,
                              const geometry_msgs::msg::Point& c) {
        marker->points.push_back(a);
        marker->points.push_back(b);
        marker->points.push_back(c);
    };

    visualization_msgs::msg::Marker body = make_marker(
        "robot_body", 0, visualization_msgs::msg::Marker::CUBE);
    body.pose.position = toPoint(position);
    body.pose.orientation = orientation;
    body.scale.x = parameters_.body_size_m.x() * 0.92;
    body.scale.y = parameters_.body_size_m.y() * 0.88;
    body.scale.z = parameters_.body_size_m.z() * 0.70;
    SetColor(&body, Color{0.14f, 0.17f, 0.20f, 0.16f});
    marker_array.markers.push_back(body);

    visualization_msgs::msg::Marker com = make_marker(
        "robot_body", 1, visualization_msgs::msg::Marker::SPHERE);
    com.pose.position = toPoint(position);
    com.scale.x = 0.032;
    com.scale.y = 0.032;
    com.scale.z = 0.032;
    SetColor(&com, Color{1.00f, 0.79f, 0.25f, 0.95f});
    marker_array.markers.push_back(com);

    visualization_msgs::msg::Marker body_axes = make_marker(
        "robot_body", 2, visualization_msgs::msg::Marker::LINE_LIST);
    body_axes.scale.x = 0.004;
    const Vec3 axis_origin = position;
    const double axis_length = std::max(
        0.10, 0.45 * std::max({parameters_.body_size_m.x(), parameters_.body_size_m.y(), parameters_.body_size_m.z()}));
    const Vec3 x_axis_end = axis_origin + body_to_world * Vec3(axis_length, 0.0, 0.0);
    const Vec3 y_axis_end = axis_origin + body_to_world * Vec3(0.0, axis_length, 0.0);
    const Vec3 z_axis_end = axis_origin + body_to_world * Vec3(0.0, 0.0, axis_length);
    body_axes.points.push_back(toPoint(axis_origin));
    body_axes.points.push_back(toPoint(x_axis_end));
    body_axes.points.push_back(toPoint(axis_origin));
    body_axes.points.push_back(toPoint(y_axis_end));
    body_axes.points.push_back(toPoint(axis_origin));
    body_axes.points.push_back(toPoint(z_axis_end));
    body_axes.colors.resize(6);
    body_axes.colors[0].r = 1.0f;
    body_axes.colors[0].a = 0.95f;
    body_axes.colors[1].r = 1.0f;
    body_axes.colors[1].a = 0.95f;
    body_axes.colors[2].g = 1.0f;
    body_axes.colors[2].a = 0.95f;
    body_axes.colors[3].g = 1.0f;
    body_axes.colors[3].a = 0.95f;
    body_axes.colors[4].b = 1.0f;
    body_axes.colors[4].a = 0.95f;
    body_axes.colors[5].b = 1.0f;
    body_axes.colors[5].a = 0.95f;
    marker_array.markers.push_back(body_axes);

    visualization_msgs::msg::Marker text = make_marker(
        "robot_body", 3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.pose.position = toPoint(bodyPointToWorld(
        Vec3(0.0, 0.0, parameters_.body_size_m.z() * 1.3), position, body_to_world));
    text.scale.z = 0.05;
    text.text = makeStatusText(context, state_label, position, velocity);
    SetColor(&text, Color{0.94f, 0.96f, 0.99f, 0.95f});
    marker_array.markers.push_back(text);

    visualization_msgs::msg::Marker hip_list = make_marker(
        "robot_joints", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
    hip_list.scale.x = 0.018;
    hip_list.scale.y = 0.018;
    hip_list.scale.z = 0.018;
    SetColor(&hip_list, Color{0.84f, 0.87f, 0.91f, 0.48f});

    if (!context.isCalibrated()) {
        for (int leg = 0; leg < NumLeg; ++leg) {
            actual_foot_trails_[leg].clear();
        }
    }

    for (int leg = 0; leg < NumLeg; ++leg) {
        const Vec3 hip_body = parameters_.hip_mounts_in_body[leg];
        const Vec3 knee_body = hip_body + kneeInHip(actual_q.col(leg), leg);
        const Vec3 foot_actual_body = context.robotModel->forwardKinematics(
            actual_q.col(leg), leg, FrameType::BODY);
        const Vec3 foot_command_body = context.robotModel->forwardKinematics(
            command_q.col(leg), leg, FrameType::BODY);

        const Vec3 hip_world = bodyPointToWorld(hip_body, position, body_to_world);
        const Vec3 knee_world = bodyPointToWorld(knee_body, position, body_to_world);
        const Vec3 foot_actual_world = bodyPointToWorld(foot_actual_body, position, body_to_world);
        const Vec3 foot_command_world = bodyPointToWorld(foot_command_body, position, body_to_world);

        hip_list.points.push_back(toPoint(hip_world));

        visualization_msgs::msg::Marker leg_marker = make_marker(
            "leg_actual", leg, visualization_msgs::msg::Marker::LINE_STRIP);
        leg_marker.scale.x = 0.006;
        add_line_points(&leg_marker, toPoint(hip_world), toPoint(knee_world), toPoint(foot_actual_world));
        SetColor(&leg_marker, LegColor(leg, 0.24f));
        marker_array.markers.push_back(leg_marker);

        visualization_msgs::msg::Marker foot_actual = make_marker(
            "foot_actual", leg, visualization_msgs::msg::Marker::SPHERE);
        foot_actual.pose.position = toPoint(foot_actual_world);
        foot_actual.scale.x = context.contact(leg) ? 0.032 : 0.024;
        foot_actual.scale.y = foot_actual.scale.x;
        foot_actual.scale.z = foot_actual.scale.x;
        SetColor(&foot_actual, LegColor(leg, context.contact(leg) ? 0.92f : 0.50f));
        marker_array.markers.push_back(foot_actual);

        visualization_msgs::msg::Marker foot_target = make_marker(
            "foot_target", leg, visualization_msgs::msg::Marker::SPHERE);
        foot_target.pose.position = toPoint(foot_command_world);
        foot_target.scale.x = 0.016;
        foot_target.scale.y = 0.016;
        foot_target.scale.z = 0.016;
        SetColor(&foot_target, LegColor(leg, 0.22f));
        marker_array.markers.push_back(foot_target);

        appendTrail(&actual_foot_trails_[leg], toPoint(foot_actual_world), stamp);

        visualization_msgs::msg::Marker actual_trail = make_marker(
            "foot_trail_actual", leg, visualization_msgs::msg::Marker::LINE_STRIP);
        actual_trail.scale.x = 0.0045;
        for (const auto& timed_point : actual_foot_trails_[leg]) {
            actual_trail.points.push_back(timed_point.point);
        }
        SetColor(&actual_trail, LegColor(leg, 0.92f));
        marker_array.markers.push_back(actual_trail);
    }

    marker_array.markers.push_back(hip_list);
    marker_pub_->publish(marker_array);
}

Vec34 VisualizationPublisher::commandJointAngles(const UserLowlevel::LowlevelCmd& low_cmd) const {
    Vec34 q_cmd = Vec34::Zero();
    for (int leg = 0; leg < NumLeg; ++leg) {
        q_cmd(0, leg) = static_cast<double>(low_cmd.motorCmd[leg * 3 + 0].q);
        q_cmd(1, leg) = static_cast<double>(low_cmd.motorCmd[leg * 3 + 1].q);
        q_cmd(2, leg) = static_cast<double>(low_cmd.motorCmd[leg * 3 + 2].q);
    }
    return q_cmd;
}

Vec3 VisualizationPublisher::kneeInHip(const Vec3& q_user, int leg_id) const {
    Vec3 q = q_user;
    if (IsFrontLeg(leg_id)) {
        q(0) = -q(0);
    }
    if (IsLeftLeg(leg_id)) {
        q(1) = -q(1);
        q(2) = -q(2);
    }

    const double q0 = q(0);
    const double q1 = q(1);
    const double x1 = parameters_.l1 * std::cos(q1);
    const double z1 = parameters_.l1 * std::sin(q1);
    const double y_off = IsLeftLeg(leg_id) ? -parameters_.l0 : parameters_.l0;

    Vec3 knee;
    knee.x() = x1;
    knee.y() = y_off * std::cos(q0) - z1 * std::sin(q0);
    knee.z() = y_off * std::sin(q0) + z1 * std::cos(q0);
    return knee;
}

Vec3 VisualizationPublisher::controllerToVizPoint(const Vec3& controller_point) const {
    return kControllerToVizReflection * controller_point;
}

Vec3 VisualizationPublisher::controllerToVizPosition(const Vec3& controller_position) const {
    return controllerToVizPoint(controller_position - controller_position_origin_);
}

Vec3 VisualizationPublisher::bodyPointToWorld(const Vec3& body_point,
                                              const Vec3& body_position,
                                              const RotMat& body_to_world) const {
    return body_position + body_to_world * controllerToVizPoint(body_point);
}

geometry_msgs::msg::Point VisualizationPublisher::toPoint(const Vec3& value) const {
    geometry_msgs::msg::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

geometry_msgs::msg::Quaternion VisualizationPublisher::toQuaternion(const RotMat& rotation) const {
    Eigen::Quaterniond q(rotation);
    q.normalize();

    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = q.w();
    quaternion.x = q.x();
    quaternion.y = q.y();
    quaternion.z = q.z();
    return quaternion;
}

void VisualizationPublisher::appendTrail(std::deque<TimedPoint>* trail,
                                         const geometry_msgs::msg::Point& point,
                                         const rclcpp::Time& stamp) const {
    if (!trail->empty()) {
        const geometry_msgs::msg::Point& last = trail->back().point;
        const double dx = point.x - last.x;
        const double dy = point.y - last.y;
        const double dz = point.z - last.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < kTrailDistanceThresholdM) {
            while (!trail->empty() &&
                   (stamp - trail->front().stamp).to_chrono<std::chrono::nanoseconds>() > kFootTrailWindow) {
                trail->pop_front();
            }
            return;
        }
    }

    trail->push_back(TimedPoint{point, stamp});
    while (!trail->empty() &&
           (stamp - trail->front().stamp).to_chrono<std::chrono::nanoseconds>() > kFootTrailWindow) {
        trail->pop_front();
    }
    while (static_cast<int>(trail->size()) > kFootTrailMaxSize) {
        trail->pop_front();
    }
}

std::string VisualizationPublisher::makeStatusText(const ControllerContext& context,
                                                   const std::string& state_label,
                                                   const Vec3& position,
                                                   const Vec3& velocity) const {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2);
    stream << "state: " << state_label
           << "  calib: " << (context.isCalibrated() ? "yes" : "no")
           << "\n";
    stream << "p=[" << position.x() << ", " << position.y() << ", " << position.z() << "]"
           << "\n";
    stream << "v=[" << velocity.x() << ", " << velocity.y() << ", " << velocity.z() << "]"
           << "\n";
    stream << "contact=[" << context.contact(0) << ", " << context.contact(1) << ", "
           << context.contact(2) << ", " << context.contact(3) << "]"
           << " phase=[" << context.phase(0) << ", " << context.phase(1) << ", "
           << context.phase(2) << ", " << context.phase(3) << "]";
    return stream.str();
}

}  // namespace qr_guide
