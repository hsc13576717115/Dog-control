#include "runtime/VisualizationPublisher.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common/enumClass.h"

namespace qr_guide {

namespace {

constexpr char kWorldFrameId[] = "odom";
constexpr auto kVisualizationInterval = std::chrono::milliseconds(33);
constexpr double kTrailDistanceThresholdM = 0.003;
constexpr double kBodyPathDistanceThresholdM = 0.01;

struct Color {
    float r;
    float g;
    float b;
    float a;
};

Color LegColor(int leg_id, float alpha) {
    static constexpr Color kColors[NumLeg] = {
        {0.96f, 0.52f, 0.24f, 1.0f},  // FR
        {0.17f, 0.76f, 0.93f, 1.0f},  // FL
        {0.55f, 0.86f, 0.33f, 1.0f},  // RR
        {0.96f, 0.35f, 0.66f, 1.0f},  // RL
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
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/qr_guide/visualization/marker_array", rclcpp::QoS(10));
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
    const Vec3 position = context.estimator->getPosition();
    const Vec3 velocity = context.estimator->getVelocity();
    const RotMat body_to_world = context.lowState->getRotMat();
    const geometry_msgs::msg::Quaternion orientation = toQuaternion(context.lowState->imu);

    publishOdometry(stamp, context, position, body_to_world);
    publishBodyPath(stamp, position, orientation);
    publishMarkers(stamp, context, state_label, position, velocity, body_to_world, orientation);
}

void VisualizationPublisher::publishOdometry(const rclcpp::Time& stamp,
                                             const ControllerContext& context,
                                             const Vec3& position,
                                             const RotMat& body_to_world) const {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = kWorldFrameId;
    odom.child_frame_id = "base_link_est";
    odom.pose.pose.position = toPoint(position);
    odom.pose.pose.orientation = toQuaternion(context.lowState->imu);

    const Vec3 velocity_body = body_to_world.transpose() * context.estimator->getVelocity();
    odom.twist.twist.linear.x = velocity_body.x();
    odom.twist.twist.linear.y = velocity_body.y();
    odom.twist.twist.linear.z = velocity_body.z();
    odom.twist.twist.angular.x = context.lowState->imu.gyroscope[0];
    odom.twist.twist.angular.y = context.lowState->imu.gyroscope[1];
    odom.twist.twist.angular.z = context.lowState->imu.gyroscope[2];
    odom_pub_->publish(odom);
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

    appendBodyPose(pose);
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
    body.scale.x = parameters_.body_size_m.x();
    body.scale.y = parameters_.body_size_m.y();
    body.scale.z = parameters_.body_size_m.z();
    SetColor(&body, Color{0.11f, 0.17f, 0.26f, 0.78f});
    marker_array.markers.push_back(body);

    visualization_msgs::msg::Marker com = make_marker(
        "robot_body", 1, visualization_msgs::msg::Marker::SPHERE);
    com.pose.position = toPoint(position);
    com.scale.x = 0.05;
    com.scale.y = 0.05;
    com.scale.z = 0.05;
    SetColor(&com, Color{0.99f, 0.84f, 0.33f, 0.95f});
    marker_array.markers.push_back(com);

    visualization_msgs::msg::Marker velocity_arrow = make_marker(
        "robot_body", 2, visualization_msgs::msg::Marker::ARROW);
    const Vec3 arrow_start_body(0.0, 0.0, parameters_.body_size_m.z() * 0.65);
    const Vec3 arrow_start = bodyPointToWorld(arrow_start_body, position, body_to_world);
    const Vec3 arrow_end = arrow_start + velocity * 0.25;
    velocity_arrow.points.push_back(toPoint(arrow_start));
    velocity_arrow.points.push_back(toPoint(arrow_end));
    velocity_arrow.scale.x = 0.018;
    velocity_arrow.scale.y = 0.035;
    velocity_arrow.scale.z = 0.05;
    SetColor(&velocity_arrow, Color{0.99f, 0.75f, 0.20f, 0.95f});
    marker_array.markers.push_back(velocity_arrow);

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
    hip_list.scale.x = 0.032;
    hip_list.scale.y = 0.032;
    hip_list.scale.z = 0.032;
    SetColor(&hip_list, Color{0.82f, 0.86f, 0.91f, 0.90f});

    if (!context.isCalibrated()) {
        for (int leg = 0; leg < NumLeg; ++leg) {
            actual_foot_trails_[leg].clear();
            command_foot_trails_[leg].clear();
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
        leg_marker.scale.x = 0.022;
        add_line_points(&leg_marker, toPoint(hip_world), toPoint(knee_world), toPoint(foot_actual_world));
        SetColor(&leg_marker, LegColor(leg, 0.95f));
        marker_array.markers.push_back(leg_marker);

        visualization_msgs::msg::Marker foot_actual = make_marker(
            "foot_actual", leg, visualization_msgs::msg::Marker::SPHERE);
        foot_actual.pose.position = toPoint(foot_actual_world);
        foot_actual.scale.x = context.contact(leg) ? 0.055 : 0.042;
        foot_actual.scale.y = foot_actual.scale.x;
        foot_actual.scale.z = foot_actual.scale.x;
        SetColor(&foot_actual, LegColor(leg, context.contact(leg) ? 0.98f : 0.55f));
        marker_array.markers.push_back(foot_actual);

        visualization_msgs::msg::Marker foot_target = make_marker(
            "foot_target", leg, visualization_msgs::msg::Marker::CUBE);
        foot_target.pose.position = toPoint(foot_command_world);
        foot_target.scale.x = 0.032;
        foot_target.scale.y = 0.032;
        foot_target.scale.z = 0.032;
        SetColor(&foot_target, LegColor(leg, 0.35f));
        marker_array.markers.push_back(foot_target);

        appendTrail(&actual_foot_trails_[leg], toPoint(foot_actual_world));
        appendTrail(&command_foot_trails_[leg], toPoint(foot_command_world));

        visualization_msgs::msg::Marker actual_trail = make_marker(
            "foot_trail_actual", leg, visualization_msgs::msg::Marker::LINE_STRIP);
        actual_trail.scale.x = 0.012;
        actual_trail.points.assign(actual_foot_trails_[leg].begin(), actual_foot_trails_[leg].end());
        SetColor(&actual_trail, LegColor(leg, 0.82f));
        marker_array.markers.push_back(actual_trail);

        visualization_msgs::msg::Marker command_trail = make_marker(
            "foot_trail_command", leg, visualization_msgs::msg::Marker::LINE_STRIP);
        command_trail.scale.x = 0.007;
        command_trail.points.assign(command_foot_trails_[leg].begin(), command_foot_trails_[leg].end());
        SetColor(&command_trail, LegColor(leg, 0.28f));
        marker_array.markers.push_back(command_trail);
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

Vec3 VisualizationPublisher::bodyPointToWorld(const Vec3& body_point,
                                              const Vec3& body_position,
                                              const RotMat& body_to_world) const {
    return body_position + body_to_world * body_point;
}

geometry_msgs::msg::Point VisualizationPublisher::toPoint(const Vec3& value) const {
    geometry_msgs::msg::Point point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

geometry_msgs::msg::Quaternion VisualizationPublisher::toQuaternion(const IMU& imu) const {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = imu.quaternion[0];
    quaternion.x = imu.quaternion[1];
    quaternion.y = imu.quaternion[2];
    quaternion.z = imu.quaternion[3];
    const double norm = std::sqrt(
        quaternion.w * quaternion.w + quaternion.x * quaternion.x +
        quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    if (norm > 1e-9) {
        quaternion.w /= norm;
        quaternion.x /= norm;
        quaternion.y /= norm;
        quaternion.z /= norm;
    } else {
        quaternion.w = 1.0;
        quaternion.x = 0.0;
        quaternion.y = 0.0;
        quaternion.z = 0.0;
    }
    return quaternion;
}

void VisualizationPublisher::appendTrail(std::deque<geometry_msgs::msg::Point>* trail,
                                         const geometry_msgs::msg::Point& point) const {
    if (!trail->empty()) {
        const geometry_msgs::msg::Point& last = trail->back();
        const double dx = point.x - last.x;
        const double dy = point.y - last.y;
        const double dz = point.z - last.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < kTrailDistanceThresholdM) {
            return;
        }
    }

    trail->push_back(point);
    while (static_cast<int>(trail->size()) > kFootTrailMaxSize) {
        trail->pop_front();
    }
}

void VisualizationPublisher::appendBodyPose(const geometry_msgs::msg::PoseStamped& pose) {
    if (!body_path_.poses.empty()) {
        const auto& last = body_path_.poses.back().pose.position;
        const double dx = pose.pose.position.x - last.x;
        const double dy = pose.pose.position.y - last.y;
        const double dz = pose.pose.position.z - last.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < kBodyPathDistanceThresholdM) {
            return;
        }
    }

    body_path_.poses.push_back(pose);
    while (static_cast<int>(body_path_.poses.size()) > kBodyPathMaxSize) {
        body_path_.poses.erase(body_path_.poses.begin());
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
