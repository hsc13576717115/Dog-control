#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace custom_dog_control {
namespace {

// 限制速度参考领先实测同向速度的幅度，避免机器人尚未跟上时目标位姿持续跑远。
// 减速请求可直接通过；实测速度反向时以零作为基线。这不是按 dt 限制加速度的 Slew。
double LimitAcceleratingReference(
    double requested, double measured, double max_lead) {
  if (requested > 0.0) {
    return std::min(requested, std::max(0.0, measured) + max_lead);
  }
  if (requested < 0.0) {
    return std::max(requested, std::min(0.0, measured) - max_lead);
  }
  return 0.0;
}

}  // namespace

controller_interface::return_type NmpcWbcController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const double now_seconds = time.seconds();
  // 估计、限速及 WBC 使用有界步长；诊断仍记录真实周期，以免掩盖调度抖动或超时。
  const double dt = std::clamp(period.seconds(), 1e-4, 0.02);
  control_period_ms_ = period.seconds() * 1000.0;
  control_timing_.Add(control_period_ms_);
  ReadHardwareState();
  if (IsRealHardware()) {
    io_timing_.Add(io_period_ms_);
  }
  // 回调只更新输入缓冲；本周期读取快照后统一仲裁速度、推进状态和写入硬件。
  imu_sample_ = *imu_buffer_.readFromRT();
  const JoyInput joy = *joy_buffer_.readFromRT();
  if (precision_) {
    HybridJointCommand output;
    const bool stop = physical_estop_ || joy.requested_mode == RequestedMode::ESTOP;
    if (precision_->Update(now_seconds, dt, joints_state_, imu_sample_, stop, output)) {
      WriteHybridCommand(output);
    } else {
      // Do not command the flat stand-up pose after a precision fault.
      for (std::size_t i = 0; i < kJointCount; ++i) {
        output.position[i] = std::isfinite(joints_state_.position[i])
                                 ? joints_state_.position[i] : 0.0;
        output.kd[i] = safe_damping_;
      }
      WriteHybridCommand(output);
    }
    return controller_interface::return_type::OK;
  }
  const VelocityCommand command = SelectVelocityCommand(now_seconds, dt);

  // 估计器先使用上一份有效策略的接触计划；首次获得策略前按四足支撑初始化。
  // planned contacts 是模型假设，不能当作真实足端触地测量。
  std::size_t planned_mode = last_policy_.valid ? last_policy_.mode : kStanceMode;
  const auto contacts = ContactFlags(planned_mode);
  if (!IsRealHardware() && use_sim_ground_truth_) {
    estimate_ = *ground_truth_buffer_.readFromRT();
    estimate_.valid =
        estimate_.valid &&
        now_seconds - estimate_.stamp_seconds <= ground_truth_timeout_s_;
  } else {
    estimate_ = estimator_->Update(joints_state_, imu_sample_, contacts, dt);
  }
  // 无效估计不推进优化器观测/目标；后面的安全检查和状态机仍会执行。
  if (estimate_.valid) {
    observation_time_ += dt;
    measured_rbd_state_ = backend_->UpdateObservation(
        estimate_, joints_state_, observation_time_, planned_mode);
    // 原始速度请求仍交给步态监督器；在新 Trot 策略确认前，给后端的速度参考保持零。
    // 否则求解器仍按四足站立约束工作时，运动目标已经向前推进，会积累跟踪误差。
    VelocityCommand backend_command = command;
    if (mode_ != OperatingMode::MPC_TROT) {
      backend_command.vx = 0.0;
      backend_command.vy = 0.0;
      backend_command.yaw = 0.0;
    } else {
      // 只按偏航角把世界系水平速度投影到机体前向/侧向，与速度指令使用同一平面基准。
      // 这里不是包含 roll/pitch 的完整三维坐标变换。
      const double yaw = estimate_.euler_zyx.x();
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double measured_vx_body =
          cos_yaw * estimate_.velocity_world.x() +
          sin_yaw * estimate_.velocity_world.y();
      const double measured_vy_body =
          -sin_yaw * estimate_.velocity_world.x() +
          cos_yaw * estimate_.velocity_world.y();
      backend_command.vx = LimitAcceleratingReference(
          command.vx, measured_vx_body, max_reference_lead_xy_m_s_);
      backend_command.vy = LimitAcceleratingReference(
          command.vy, measured_vy_body, max_reference_lead_xy_m_s_);
      backend_command.yaw = LimitAcceleratingReference(
          command.yaw, estimate_.angular_velocity_world.z(),
          max_reference_lead_yaw_rad_s_);
    }
    governed_command_ = backend_command;
    const bool target_command_is_moving =
        std::max({std::abs(backend_command.vx),
                  std::abs(backend_command.vy),
                  std::abs(backend_command.yaw)}) > 1e-3;
    // 连续运动参考至多每 50 ms 更新一次；首次目标、停车和站立漂移重锚定走事件更新。
    const bool moving_target_update_due =
        target_command_is_moving &&
        now_seconds - last_target_update_seconds_ >= 0.05;
    const double stance_anchor_error = std::hypot(
        estimate_.position.x() - stance_anchor_xy_[0],
        estimate_.position.y() - stance_anchor_xy_[1]);
    const bool stance_reanchor_due =
        mode_ == OperatingMode::MPC_STANCE && !target_command_is_moving &&
        stance_anchor_error >= stance_reanchor_distance_m_;
    // 两次更新之间保持参考不变，避免位置控制向 WBC 交接期间不断扰动求解目标。
    if (last_target_update_seconds_ < 0.0 || moving_target_update_due ||
        stance_reanchor_due ||
        (target_command_was_moving_ && !target_command_is_moving)) {
      backend_->SetVelocityCommand(
          backend_command,
          stance_reanchor_due ||
              (target_command_was_moving_ && !target_command_is_moving));
      last_target_update_seconds_ = now_seconds;
      if (!target_command_is_moving) {
        stance_anchor_xy_ = {estimate_.position.x(), estimate_.position.y()};
      }
    }
    target_command_was_moving_ = target_command_is_moving;
  }

  // 只有评估成功才替换策略缓存；序号变化才计入一次 MPC 求解耗时，
  // 因为同一份后台策略会被高频控制循环重复评估。
  PolicySample evaluated_policy;
  const bool evaluated = estimate_.valid &&
                         backend_->EvaluatePolicy(now_seconds, evaluated_policy);
  if (evaluated) {
    policy_buffer_.writeFromNonRT(evaluated_policy);
    if (evaluated_policy.sequence != last_timed_policy_sequence_) {
      mpc_timing_.Add(evaluated_policy.solve_time_ms);
      last_timed_policy_sequence_ = evaluated_policy.sequence;
    }
  }
  last_policy_ = *policy_buffer_.readFromRT();
  const bool policy_available = last_policy_.valid;
  // 后端按单调墙钟计算策略年龄，仿真时钟暂停不会把陈旧策略误判为新鲜。
  const double policy_age = backend_->policyAgeSeconds(now_seconds);

  // 起身和 MPC 活动模式执行安全检查；求解器/策略时效检查由 dynamic_mode 限定为 Trot。
  // 安全故障先于本周期的步态监督处理，防止继续发出正常运动切换请求。
  if (mode_ != OperatingMode::PASSIVE &&
      mode_ != OperatingMode::CALIBRATION &&
      mode_ != OperatingMode::FAULT) {
    SafetyInput safety_input;
    safety_input.joints = joints_state_;
    safety_input.imu = imu_sample_;
    safety_input.now_seconds = now_seconds;
    safety_input.policy_age_s = policy_age;
    safety_input.roll = estimate_.euler_zyx.z();
    safety_input.pitch = estimate_.euler_zyx.y();
    safety_input.consecutive_io_failures = consecutive_io_failures_;
    safety_input.communication_ok = communication_ok_;
    safety_input.physical_estop = physical_estop_;
    safety_input.solver_valid = backend_->solverHealthy();
    safety_input.dynamic_mode = mode_ == OperatingMode::MPC_TROT;
    if (!safety_monitor_->Evaluate(safety_input)) {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Safety fault: %s",
          safety_monitor_->reason().c_str());
      TransitionTo(OperatingMode::FAULT, now_seconds);
    }
  }

  if (mode_ != OperatingMode::FAULT) {
    UpdateLocomotionSupervisor(
        now_seconds, command, joy, last_policy_, policy_age);
  }

  // 状态机负责起身 PD、WBC 输出和短暂失败回退；最后集中决定正常输出或安全输出。
  // state_ok 只说明状态机已处理本周期，不能替代对 PASSIVE/CALIBRATION/FAULT 的判断。
  HybridJointCommand hybrid_command;
  WbcOutput wbc;
  const bool state_ok = ApplyStateMachine(
      now_seconds, dt, joy, last_policy_, policy_available,
      hybrid_command, wbc);
  if (!state_ok || mode_ == OperatingMode::PASSIVE ||
      mode_ == OperatingMode::CALIBRATION ||
      mode_ == OperatingMode::FAULT) {
    WriteSafeCommand();
  } else {
    WriteHybridCommand(hybrid_command);
  }

  PublishDiagnostics(time, estimate_, last_policy_, wbc);
  // FAULT 由内部模式和安全命令处理；这里的 OK 表示完成 update，不表示机器人无故障。
  return controller_interface::return_type::OK;
}

}  // namespace custom_dog_control

// 与插件描述 XML 配合，使 controller_manager 能按插件名创建本控制器。
PLUGINLIB_EXPORT_CLASS(
    custom_dog_control::NmpcWbcController,
    controller_interface::ControllerInterface)
