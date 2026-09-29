#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace custom_dog_control {
namespace {

diagnostic_msgs::msg::KeyValue KeyValue(
    const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

std::string Number(double value) {
  std::ostringstream stream;
  stream.precision(6);
  stream << value;
  return stream.str();
}

Eigen::Quaterniond QuaternionFromZyx(const Eigen::Vector3d& zyx) {
  return Eigen::AngleAxisd(zyx.x(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(zyx.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(zyx.z(), Eigen::Vector3d::UnitX());
}

}  // namespace

void NmpcWbcController::PublishDiagnostics(
    const rclcpp::Time& stamp, const EstimatedState& estimate,
    const PolicySample& policy, const WbcOutput& wbc) {
  // 诊断按 ROS 时间限频到约 10 Hz；统计窗口在控制循环中持续采样。
  // 这里仍有消息分配、排序和发布开销，并非独立的实时安全发布线程。
  if (last_diagnostics_seconds_ >= 0.0 &&
      stamp.seconds() - last_diagnostics_seconds_ < 0.10) {
    return;
  }
  last_diagnostics_seconds_ = stamp.seconds();

  std_msgs::msg::String mode_message;
  mode_message.data = std::string(ToString(mode_));
  mode_publisher_->publish(mode_message);

  std_msgs::msg::Float64MultiArray contact_message;
  // 发布的是策略接触计划；无有效策略时显示全支撑，不表示检测到了触地。
  const auto contacts = ContactFlags(policy.valid ? policy.mode : kStanceMode);
  contact_message.data.reserve(kLegCount);
  for (const bool contact : contacts) {
    contact_message.data.push_back(contact ? 1.0 : 0.0);
  }
  contact_publisher_->publish(contact_message);

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "custom_dog_control/nmpc_wbc";
  status.hardware_id = IsRealHardware() ? "custom_dog_rs485" : "gazebo";
  status.level =
      mode_ == OperatingMode::FAULT
          ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
          : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message =
      mode_ == OperatingMode::FAULT && safety_monitor_->faulted()
          ? safety_monitor_->reason()
          : std::string(ToString(mode_));
  const auto control_timing = control_timing_.Snapshot();
  const auto io_timing = io_timing_.Snapshot();
  const auto mpc_timing = mpc_timing_.Snapshot();
  double wbc_tau_max_abs = 0.0;
  for (const double effort : wbc.command.effort) {
    wbc_tau_max_abs = std::max(wbc_tau_max_abs, std::abs(effort));
  }
  status.values = {
      KeyValue("mode", std::string(ToString(mode_))),
      KeyValue("trot_enabled", trot_enabled_ ? "true" : "false"),
      KeyValue(
          "gait_transition",
          gait_transition_ == GaitTransition::STARTING_TROT
              ? "STARTING_TROT"
              : (gait_transition_ == GaitTransition::STOPPING_TROT
                     ? "STOPPING_TROT"
                     : "NONE")),
      KeyValue("policy_mode", std::to_string(policy.mode)),
      KeyValue("nmpc_backend", "legged::LeggedInterface"),
      KeyValue("wbc_backend", "legged::WeightedWbc"),
      KeyValue(
          "legged_control_revision",
          "a7f381c0367e98e31c01336e678eef47e304d40d"),
      KeyValue("control_period_ms", Number(control_period_ms_)),
      KeyValue("control_period_mean_ms", Number(control_timing.mean)),
      KeyValue("control_period_p95_ms", Number(control_timing.p95)),
      KeyValue("control_period_p99_ms", Number(control_timing.p99)),
      KeyValue("io_period_ms", Number(io_period_ms_)),
      KeyValue("io_period_p99_ms", Number(io_timing.p99)),
      KeyValue("io_timeout_count", Number(io_timeout_count_)),
      KeyValue("communication_ok", communication_ok_ ? "true" : "false"),
      KeyValue("base_x_m", Number(estimate.position.x())),
      KeyValue("base_y_m", Number(estimate.position.y())),
      KeyValue("base_z_m", Number(estimate.position.z())),
      KeyValue("base_yaw_rad", Number(estimate.euler_zyx.x())),
      KeyValue("base_roll_rad", Number(estimate.euler_zyx.z())),
      KeyValue("base_pitch_rad", Number(estimate.euler_zyx.y())),
      KeyValue("base_vx_mps", Number(estimate.velocity_world.x())),
      KeyValue("base_vy_mps", Number(estimate.velocity_world.y())),
      KeyValue("base_vz_mps", Number(estimate.velocity_world.z())),
      KeyValue("requested_vx_mps", Number(limited_command_.vx)),
      KeyValue("requested_vy_mps", Number(limited_command_.vy)),
      KeyValue("requested_yaw_rad_s", Number(limited_command_.yaw)),
      KeyValue("governed_vx_mps", Number(governed_command_.vx)),
      KeyValue("governed_vy_mps", Number(governed_command_.vy)),
      KeyValue("governed_yaw_rad_s", Number(governed_command_.yaw)),
      KeyValue("fr_hip_position_rad", Number(joints_state_.position[0])),
      KeyValue("fr_thigh_position_rad", Number(joints_state_.position[1])),
      KeyValue("fr_calf_position_rad", Number(joints_state_.position[2])),
      KeyValue("fr_hip_velocity_rad_s", Number(joints_state_.velocity[0])),
      KeyValue("fr_thigh_velocity_rad_s", Number(joints_state_.velocity[1])),
      KeyValue("fr_calf_velocity_rad_s", Number(joints_state_.velocity[2])),
      KeyValue("wbc_q_fr_hip_rad", Number(wbc.command.position[0])),
      KeyValue("wbc_q_fr_thigh_rad", Number(wbc.command.position[1])),
      KeyValue("wbc_q_fr_calf_rad", Number(wbc.command.position[2])),
      KeyValue("wbc_qd_fr_hip_rad_s", Number(wbc.command.velocity[0])),
      KeyValue("wbc_qd_fr_thigh_rad_s", Number(wbc.command.velocity[1])),
      KeyValue("wbc_qd_fr_calf_rad_s", Number(wbc.command.velocity[2])),
      KeyValue("mpc_policy_age_s", Number(backend_->policyAgeSeconds(stamp.seconds()))),
      KeyValue("mpc_solver_healthy", backend_->solverHealthy() ? "true" : "false"),
      KeyValue("mpc_last_error", backend_->lastError()),
      KeyValue("mpc_solve_ms", Number(backend_->lastSolveTimeMs())),
      KeyValue("mpc_solve_mean_ms", Number(mpc_timing.mean)),
      KeyValue("mpc_solve_p95_ms", Number(mpc_timing.p95)),
      KeyValue("mpc_solve_p99_ms", Number(mpc_timing.p99)),
      KeyValue("wbc_solve_ms", Number(wbc.solve_time_ms)),
      KeyValue("wbc_valid", wbc.valid ? "true" : "false"),
      KeyValue(
          "wbc_consecutive_failures",
          std::to_string(consecutive_wbc_failures_)),
      KeyValue("wbc_tau_max_abs", Number(wbc_tau_max_abs)),
      KeyValue("wbc_tau_fr_hip", Number(wbc.command.effort[0])),
      KeyValue("wbc_tau_fr_thigh", Number(wbc.command.effort[1])),
      KeyValue("wbc_tau_fr_calf", Number(wbc.command.effort[2])),
      KeyValue("wbc_equality_residual", Number(wbc.equality_residual)),
      KeyValue("wbc_inequality_violation", Number(wbc.inequality_violation)),
      KeyValue("policy_fz_fr", Number(policy.input[2])),
      KeyValue("policy_fz_fl", Number(policy.input[5])),
      KeyValue("policy_fz_rr", Number(policy.input[8])),
      KeyValue("policy_fz_rl", Number(policy.input[11]))};
  diagnostics.status.push_back(std::move(status));
  diagnostics_publisher_->publish(diagnostics);

  if (!estimate.valid) {
    return;
  }
  const Eigen::Quaterniond quaternion = QuaternionFromZyx(estimate.euler_zyx);
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base";
  odometry.pose.pose.position.x = estimate.position.x();
  odometry.pose.pose.position.y = estimate.position.y();
  odometry.pose.pose.position.z = estimate.position.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  // Odometry 的 pose 位于 odom，twist 位于 child_frame_id=base；
  // 估计器输出的是世界系速度，发布前必须旋转到机体系。
  const Eigen::Vector3d velocity_body =
      quaternion.inverse() * estimate.velocity_world;
  const Eigen::Vector3d angular_body =
      quaternion.inverse() * estimate.angular_velocity_world;
  odometry.twist.twist.linear.x = velocity_body.x();
  odometry.twist.twist.linear.y = velocity_body.y();
  odometry.twist.twist.linear.z = velocity_body.z();
  odometry.twist.twist.angular.x = angular_body.x();
  odometry.twist.twist.angular.y = angular_body.y();
  odometry.twist.twist.angular.z = angular_body.z();
  odom_publisher_->publish(odometry);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = "base";
  transform.transform.translation.x = estimate.position.x();
  transform.transform.translation.y = estimate.position.y();
  transform.transform.translation.z = estimate.position.z();
  transform.transform.rotation = odometry.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace custom_dog_control
