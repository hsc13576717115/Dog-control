#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>

namespace custom_dog_control {
namespace {

bool Button(const sensor_msgs::msg::Joy& message, std::size_t index) {
  return index < message.buttons.size() && message.buttons[index] != 0;
}

double Axis(const sensor_msgs::msg::Joy& message, std::size_t index) {
  return index < message.axes.size() ? message.axes[index] : 0.0;
}

}  // namespace

void NmpcWbcController::ImuCallback(
    const sensor_msgs::msg::Imu::SharedPtr message) {
  ImuSample sample;
  sample.orientation_wxyz = {
      message->orientation.w, message->orientation.x,
      message->orientation.y, message->orientation.z};
  sample.angular_velocity = {
      message->angular_velocity.x, message->angular_velocity.y,
      message->angular_velocity.z};
  sample.linear_acceleration = {
      message->linear_acceleration.x, message->linear_acceleration.y,
      message->linear_acceleration.z};
  const rclcpp::Time sensor_stamp(message->header.stamp);
  sample.stamp_seconds =
      sensor_stamp.nanoseconds() > 0
          ? sensor_stamp.seconds()
          : get_node()->get_clock()->now().seconds();
  sample.valid = std::isfinite(message->orientation.w) &&
                 std::isfinite(message->linear_acceleration.x);
  imu_buffer_.writeFromNonRT(sample);
}

void NmpcWbcController::GroundTruthCallback(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  EstimatedState sample;
  sample.position = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
  Eigen::Quaterniond orientation(
      message->pose.pose.orientation.w,
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z);
  if (std::isfinite(orientation.norm()) && orientation.norm() > 1e-6) {
    orientation.normalize();
    sample.euler_zyx = EulerZyxFromRotation(orientation.toRotationMatrix());
  }
  sample.velocity_world = {
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z};
  sample.angular_velocity_world = {
      message->twist.twist.angular.x,
      message->twist.twist.angular.y,
      message->twist.twist.angular.z};
  sample.stamp_seconds = get_node()->get_clock()->now().seconds();
  sample.valid = sample.position.allFinite() &&
                 sample.euler_zyx.allFinite() &&
                 sample.velocity_world.allFinite() &&
                 sample.angular_velocity_world.allFinite();
  ground_truth_buffer_.writeFromNonRT(sample);
}

void NmpcWbcController::JoyCallback(
    const sensor_msgs::msg::Joy::SharedPtr message) {
  JoyInput input;
  input.stamp_seconds = get_node()->get_clock()->now().seconds();
  input.velocity.vx = Axis(*message, 1) * velocity_limits_.vx;
  input.velocity.vy = LegacyJoyLateralToRep103(
      Axis(*message, 0) * velocity_limits_.vy, legacy_joy_y_right_);
  input.velocity.yaw = Axis(*message, 3) * velocity_limits_.yaw;
  input.velocity.stamp_seconds = input.stamp_seconds;
  input.velocity.active = true;

  if (Button(*message, 3)) {
    input.requested_mode = RequestedMode::ESTOP;
  } else if (Button(*message, 1)) {
    input.requested_mode = RequestedMode::PASSIVE;
  } else if (Button(*message, 7)) {
    input.requested_mode = RequestedMode::CALIBRATE_AND_STAND;
  } else if (Button(*message, 2)) {
    input.requested_mode = RequestedMode::TROT;
  } else if (Button(*message, 0)) {
    input.requested_mode = RequestedMode::STANCE;
  }
  const double magnitude =
      std::max({std::abs(input.velocity.vx), std::abs(input.velocity.vy),
                std::abs(input.velocity.yaw)});
  // Mode buttons must not make a zero-valued joystick override /cmd_vel.
  // The request is consumed independently through requested_mode and its stamp.
  input.active = magnitude > 0.01;
  joy_buffer_.writeFromNonRT(input);
}

void NmpcWbcController::CmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr message) {
  VelocityCommand command;
  command.vx = message->linear.x;
  command.vy = message->linear.y;
  command.yaw = message->angular.z;
  command.stamp_seconds = get_node()->get_clock()->now().seconds();
  command.active = true;
  cmd_vel_buffer_.writeFromNonRT(ClampVelocity(command, velocity_limits_));
}

VelocityCommand NmpcWbcController::SelectVelocityCommand(
    double now_seconds, double dt) {
  const JoyInput joy = *joy_buffer_.readFromRT();
  const VelocityCommand cmd_vel = *cmd_vel_buffer_.readFromRT();
  VelocityCommand target;
  if (joy.active && now_seconds - joy.stamp_seconds <= joy_timeout_s_) {
    target = joy.velocity;
  } else if (cmd_vel.active &&
             now_seconds - cmd_vel.stamp_seconds <= command_timeout_s_) {
    target = cmd_vel;
  }
  target = ClampVelocity(target, velocity_limits_);
  limited_command_ = SlewVelocity(limited_command_, target, velocity_limits_, dt);
  limited_command_.stamp_seconds = now_seconds;
  // A source timeout or zero command starts a controlled deceleration; it
  // must not deactivate locomotion while the slew-limited command is still
  // moving. Otherwise the supervisor inserts STANCE at running speed.
  limited_command_.active =
      target.active || !IsBelowTrotExitThreshold(limited_command_);
  return limited_command_;
}

}  // namespace custom_dog_control
