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
  // 回调只生成输入快照，状态机与电机输出统一由 update 消费。
  // 此处不做 TF 变换，话题需提供与机体轴一致的 IMU 数据。
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
  // 优先保留采样时刻，使通信延迟计入超时；无时间戳时才以接收时刻兜底。
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
  // 此入口对应本项目 Gazebo 真值话题，直接把 twist 当世界系速度使用。
  // 接入其他 Odometry 发布器时需核对其 twist 坐标系，不能只匹配消息类型。
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

  // 同一消息按急停、被动、标定起立、Trot、站立的顺序裁决按钮冲突。
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
  // 模式请求与速度通道独立：仅按按钮时，零摇杆不抢占 /cmd_vel。
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
  // 新鲜且有偏转的摇杆优先，其次是新鲜的 /cmd_vel；均失效则目标归零。
  if (joy.active && now_seconds - joy.stamp_seconds <= joy_timeout_s_) {
    target = joy.velocity;
  } else if (cmd_vel.active &&
             now_seconds - cmd_vel.stamp_seconds <= command_timeout_s_) {
    target = cmd_vel;
  }
  target = ClampVelocity(target, velocity_limits_);
  limited_command_ = SlewVelocity(limited_command_, target, velocity_limits_, dt);
  limited_command_.stamp_seconds = now_seconds;
  // 输入超时先按加速度限制减速。斜坡输出仍在运动时保持 active，
  // 避免步态监督器在速度尚未降下来时就插入全支撑。
  limited_command_.active =
      target.active || !IsBelowTrotExitThreshold(limited_command_);
  return limited_command_;
}

}  // namespace custom_dog_control
