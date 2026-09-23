#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>

#include "custom_dog_control/control/JointCommandUtils.hpp"

namespace custom_dog_control {
namespace {

double SmoothStep(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

}  // namespace

void NmpcWbcController::TransitionTo(
    OperatingMode next, double now_seconds) {
  if (mode_ == next) {
    return;
  }
  const OperatingMode previous = mode_;
  RCLCPP_INFO(
      get_node()->get_logger(), "Control mode %s -> %s",
      ToString(mode_).data(), ToString(next).data());
  mode_ = next;
  state_entered_seconds_ = now_seconds;
  if (next == OperatingMode::STAND_UP) {
    consecutive_wbc_failures_ = 0;
    stand_start_positions_ = joints_state_.position;
  } else if (next == OperatingMode::MPC_STANCE) {
    // The fixed-pose blend is only the position-control-to-WBC handoff.
    // Reapplying it after Trot would drag four planted feet back to the
    // original stand posture and can overturn the robot after a yaw command.
    stance_handoff_active_ = previous == OperatingMode::STAND_UP;
    consecutive_wbc_failures_ = 0;
    // Arm locomotion after the explicit calibration and stand-up sequence.
    // Zero velocity is still handled as MPC_STANCE by the gait supervisor.
    trot_enabled_ = true;
    backend_->RequestGait(false);
    // Position-controlled stand-up can translate and rotate the floating
    // base. Start WBC from the achieved pose instead of chasing the NMPC
    // target captured before Gazebo physics was unpaused.
    backend_->SetVelocityCommand(VelocityCommand{}, true, true);
    last_target_update_seconds_ = now_seconds;
    target_command_was_moving_ = false;
    stance_anchor_xy_ = {estimate_.position.x(), estimate_.position.y()};
  } else if (next == OperatingMode::FAULT) {
    requires_recalibration_ = true;
    trot_enabled_ = false;
    gait_transition_ = GaitTransition::NONE;
    gait_condition_since_seconds_ = -1.0;
  }
}

void NmpcWbcController::UpdateLocomotionSupervisor(
    double now_seconds, const VelocityCommand& command, const JoyInput& joy,
    const PolicySample& policy, double policy_age_seconds) {
  const RequestedMode request =
      now_seconds - joy.stamp_seconds <= joy_timeout_s_
          ? joy.requested_mode
          : RequestedMode::NONE;
  if (request == RequestedMode::TROT &&
      (mode_ == OperatingMode::MPC_STANCE ||
       mode_ == OperatingMode::MPC_TROT)) {
    trot_enabled_ = true;
  } else if (request == RequestedMode::STANCE) {
    trot_enabled_ = false;
  }

  if (mode_ != OperatingMode::MPC_STANCE &&
      mode_ != OperatingMode::MPC_TROT) {
    gait_transition_ = GaitTransition::NONE;
    gait_condition_since_seconds_ = -1.0;
    return;
  }

  const bool policy_fresh =
      policy.valid && backend_->solverHealthy() &&
      policy_age_seconds <= safety_monitor_->limits().max_policy_age_s;

  if (gait_transition_ == GaitTransition::STARTING_TROT) {
    if (!trot_enabled_ || !ExceedsTrotEntryThreshold(command)) {
      backend_->RequestGait(false);
      gait_transition_ = GaitTransition::NONE;
      gait_condition_since_seconds_ = -1.0;
    } else if (policy_fresh &&
               policy.sequence > gait_request_policy_sequence_ &&
               IsTrotMode(policy.mode)) {
      gait_transition_ = GaitTransition::NONE;
      gait_condition_since_seconds_ = -1.0;
      TransitionTo(OperatingMode::MPC_TROT, now_seconds);
    } else {
      backend_->RequestGait(true);
    }
    return;
  }

  if (gait_transition_ == GaitTransition::STOPPING_TROT) {
    if (policy_fresh &&
        policy.sequence > gait_request_policy_sequence_ &&
        policy.mode == kStanceMode) {
      gait_transition_ = GaitTransition::NONE;
      gait_condition_since_seconds_ = -1.0;
      TransitionTo(OperatingMode::MPC_STANCE, now_seconds);
    }
    return;
  }

  if (mode_ == OperatingMode::MPC_STANCE) {
    if (!trot_enabled_ || !ExceedsTrotEntryThreshold(command) ||
        !policy_fresh) {
      gait_condition_since_seconds_ = -1.0;
      return;
    }
    if (gait_condition_since_seconds_ < 0.0) {
      gait_condition_since_seconds_ = now_seconds;
    }
    if (now_seconds - gait_condition_since_seconds_ >= trot_entry_dwell_s_) {
      backend_->RequestGait(true);
      gait_request_policy_sequence_ = policy.sequence;
      gait_transition_ = GaitTransition::STARTING_TROT;
      gait_condition_since_seconds_ = -1.0;
    }
    return;
  }

  const bool stop_requested =
      !trot_enabled_ || IsBelowTrotExitThreshold(command) || !command.active;
  if (!stop_requested) {
    backend_->RequestGait(true);
    gait_condition_since_seconds_ = -1.0;
    return;
  }
  if (gait_condition_since_seconds_ < 0.0) {
    gait_condition_since_seconds_ = now_seconds;
  }
  if (now_seconds - gait_condition_since_seconds_ >= trot_exit_dwell_s_) {
    backend_->RequestGait(false);
    gait_request_policy_sequence_ = policy.sequence;
    gait_transition_ = GaitTransition::STOPPING_TROT;
    gait_condition_since_seconds_ = -1.0;
  }
}

bool NmpcWbcController::ApplyStateMachine(
    double now_seconds, double dt, const JoyInput& joy,
    const PolicySample& policy, bool policy_available,
    HybridJointCommand& command, WbcOutput& wbc) {
  const RequestedMode request =
      now_seconds - joy.stamp_seconds <= joy_timeout_s_
          ? joy.requested_mode
          : RequestedMode::NONE;

  if (request == RequestedMode::ESTOP) {
    TransitionTo(OperatingMode::FAULT, now_seconds);
  }
  if (mode_ == OperatingMode::FAULT) {
    if (request == RequestedMode::PASSIVE) {
      safety_monitor_->Reset();
      reset_calibration_pending_ = IsRealHardware();
      requires_recalibration_ = true;
      TransitionTo(OperatingMode::PASSIVE, now_seconds);
    }
    return true;
  }

  if (request == RequestedMode::PASSIVE) {
    TransitionTo(OperatingMode::PASSIVE, now_seconds);
  }
  if (mode_ == OperatingMode::PASSIVE &&
      (request == RequestedMode::CALIBRATE_AND_STAND ||
       request == RequestedMode::STANCE ||
       request == RequestedMode::TROT)) {
    if (IsRealHardware() && (requires_recalibration_ || !calibrated_)) {
      TransitionTo(OperatingMode::CALIBRATION, now_seconds);
    } else {
      TransitionTo(OperatingMode::STAND_UP, now_seconds);
    }
  }

  if (mode_ == OperatingMode::CALIBRATION) {
    if (calibrated_) {
      requires_recalibration_ = false;
      TransitionTo(OperatingMode::STAND_UP, now_seconds);
    }
    return true;
  }

  if (mode_ == OperatingMode::PASSIVE) {
    return true;
  }

  if (mode_ == OperatingMode::STAND_UP) {
    const double elapsed = now_seconds - state_entered_seconds_;
    const double duration = std::max(0.1, stand_up_duration_s_);
    // Match unitree_guide FixedStand: interpolate every joint from the
    // measured entry pose to the fixed standing pose with position PD.
    const double phase = SmoothStep(elapsed / duration);
    const double settle_phase = SmoothStep(
        (elapsed - stand_up_duration_s_) /
        std::max(0.1, stand_up_settle_duration_s_));
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const std::size_t joint_in_leg = i % kJointsPerLeg;
      command.position[i] =
          (1.0 - phase) * stand_start_positions_[i] +
          phase * stand_up_joint_positions_[i];
      command.velocity[i] = 0.0;
      command.effort[i] = 0.0;
      const double initial_kp =
          joint_in_leg == 0
              ? stand_up_hip_kp_
              : (joint_in_leg == 1 ? stand_up_leg_kp_ : stand_up_calf_kp_);
      const double settled_kp =
          joint_in_leg == 0
              ? stand_up_settle_hip_kp_
              : (joint_in_leg == 1 ? stand_up_settle_leg_kp_
                                   : stand_up_settle_calf_kp_);
      command.kp[i] =
          (1.0 - settle_phase) * initial_kp + settle_phase * settled_kp;
      command.kd[i] =
          (1.0 - settle_phase) * stand_up_kd_ +
          settle_phase * stand_up_settle_kd_;
    }
    bool joints_settled = true;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      joints_settled =
          joints_settled &&
          std::abs(joints_state_.position[i] - stand_up_joint_positions_[i]) <=
              stand_up_position_tolerance_rad_ &&
          std::abs(joints_state_.velocity[i]) <=
              stand_up_velocity_tolerance_rad_s_;
    }
    if (elapsed >= stand_up_duration_s_ + stand_up_settle_duration_s_ &&
        joints_settled &&
        estimate_.velocity_world.norm() <=
            stand_up_base_velocity_tolerance_m_s_ &&
        policy_available) {
      TransitionTo(OperatingMode::MPC_STANCE, now_seconds);
    }
    return true;
  }

  if (!policy_available || measured_rbd_state_.size() == 0) {
    return false;
  }
  wbc = backend_->ComputeWbc(policy, measured_rbd_state_, dt);
  if (!wbc.valid) {
    ++consecutive_wbc_failures_;
    if (consecutive_wbc_failures_ >= max_consecutive_wbc_failures_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "WBC failed for %d consecutive control cycles; entering FAULT",
          consecutive_wbc_failures_);
      TransitionTo(OperatingMode::FAULT, now_seconds);
      return false;
    }

    // A single active-set failure must not drop a standing robot. Hold the
    // proven position-controlled stance while NMPC/WBC recovers next cycle.
    command = PositionPdCommand(
        stand_up_joint_positions_,
        {stand_up_settle_hip_kp_, stand_up_settle_leg_kp_,
         stand_up_settle_calf_kp_},
        stand_up_settle_kd_);
    return true;
  }
  consecutive_wbc_failures_ = 0;
  command = wbc.command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    // Match legged_control's hybrid command: optimized position/velocity,
    // low joint impedance, and WBC feed-forward torque.
    command.kp[i] =
        i % kJointsPerLeg == 0 ? wbc_hip_stiffness_ : wbc_joint_stiffness_;
    command.kd[i] = wbc_joint_damping_;
  }
  if (mode_ == OperatingMode::MPC_STANCE && stance_handoff_active_) {
    const double alpha = SmoothStep(
        (now_seconds - state_entered_seconds_) /
        std::max(0.1, handoff_duration_s_));
    const auto stand_command = PositionPdCommand(
        stand_up_joint_positions_,
        {stand_up_settle_hip_kp_, stand_up_settle_leg_kp_,
         stand_up_settle_calf_kp_},
        stand_up_settle_kd_);
    command = BlendHybridCommands(stand_command, command, joints_state_, alpha);
  }
  return true;
}

}  // namespace custom_dog_control
