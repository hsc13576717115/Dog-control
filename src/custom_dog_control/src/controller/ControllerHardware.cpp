#include "custom_dog_control/controller/NmpcWbcController.hpp"

#include <algorithm>
#include <cmath>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "custom_dog_control/control/JointCommandUtils.hpp"

namespace custom_dog_control {

controller_interface::InterfaceConfiguration
NmpcWbcController::command_interface_configuration() const {
  // 真机把五分量混合命令交给电机端 PD；Gazebo 只接受 effort，由本控制器合成 PD 力矩。
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  if (IsRealHardware()) {
    configuration.names.reserve(kJointCount * 5 + 2);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/position");
      configuration.names.push_back(joint + "/velocity");
      configuration.names.push_back(joint + "/effort");
      configuration.names.push_back(joint + "/kp");
      configuration.names.push_back(joint + "/kd");
    }
    configuration.names.push_back("custom_dog/calibrate");
    configuration.names.push_back("custom_dog/emergency_stop");
  } else {
    configuration.names.reserve(kJointCount);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/effort");
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
NmpcWbcController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  if (IsRealHardware()) {
    configuration.names.reserve(kJointCount * 5 + 5);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/position");
      configuration.names.push_back(joint + "/velocity");
      configuration.names.push_back(joint + "/effort");
      configuration.names.push_back(joint + "/temperature");
      configuration.names.push_back(joint + "/valid");
    }
    configuration.names.push_back("custom_dog/calibrated");
    configuration.names.push_back("custom_dog/communication_ok");
    configuration.names.push_back("custom_dog/physical_estop");
    configuration.names.push_back("custom_dog/io_period_ms");
    configuration.names.push_back("custom_dog/io_timeout_count");
  } else {
    configuration.names.reserve(kJointCount * 3);
    for (const auto& joint : joints_) {
      configuration.names.push_back(joint + "/position");
      configuration.names.push_back(joint + "/velocity");
      configuration.names.push_back(joint + "/effort");
    }
  }
  return configuration;
}

// 管理器借出接口的顺序不属于本包契约：激活时按完整名称查找，周期路径仅访问缓存下标。
// 累积 valid 时把查找放在 && 左侧，确保一次缺失后仍检查并报告其余接口。
bool NmpcWbcController::ResolveInterfaceIndices() {
  const auto find_command = [this](const std::string& name, std::size_t& index) {
    for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (command_interfaces_[i].get_name() == name) {
        index = i;
        return true;
      }
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Missing command interface: %s", name.c_str());
    return false;
  };
  const auto find_state = [this](const std::string& name, std::size_t& index) {
    for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == name) {
        index = i;
        return true;
      }
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Missing state interface: %s", name.c_str());
    return false;
  };

  // 列顺序与下面 ReadHardwareState/WriteHybridCommand 的固定下标必须一起维护。
  const std::array<std::string, 5> real_command_types = {
      "position", "velocity", "effort", "kp", "kd"};
  const std::array<std::string, 5> real_state_types = {
      "position", "velocity", "effort", "temperature", "valid"};
  const std::array<std::string, 3> simulation_state_types = {
      "position", "velocity", "effort"};

  bool valid = true;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (IsRealHardware()) {
      for (std::size_t type = 0; type < real_command_types.size(); ++type) {
        valid = find_command(
                    joints_[joint] + "/" + real_command_types[type],
                    command_interface_indices_[joint][type]) && valid;
      }
      for (std::size_t type = 0; type < real_state_types.size(); ++type) {
        valid = find_state(
                    joints_[joint] + "/" + real_state_types[type],
                    state_interface_indices_[joint][type]) && valid;
      }
    } else {
      valid = find_command(
                  joints_[joint] + "/effort",
                  command_interface_indices_[joint][0]) && valid;
      for (std::size_t type = 0; type < simulation_state_types.size(); ++type) {
        valid = find_state(
                    joints_[joint] + "/" + simulation_state_types[type],
                    state_interface_indices_[joint][type]) && valid;
      }
    }
  }

  if (IsRealHardware()) {
    const std::array<std::string, 2> system_commands = {
        "custom_dog/calibrate", "custom_dog/emergency_stop"};
    const std::array<std::string, 5> system_states = {
        "custom_dog/calibrated", "custom_dog/communication_ok",
        "custom_dog/physical_estop", "custom_dog/io_period_ms",
        "custom_dog/io_timeout_count"};
    for (std::size_t i = 0; i < system_commands.size(); ++i) {
      valid = find_command(system_commands[i], system_command_interface_indices_[i]) && valid;
    }
    for (std::size_t i = 0; i < system_states.size(); ++i) {
      valid = find_state(system_states[i], system_state_interface_indices_[i]) && valid;
    }
  }
  return valid;
}

void NmpcWbcController::ReadHardwareState() {
  if (IsRealHardware()) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      joints_state_.position[i] = state_interfaces_[state_interface_indices_[i][0]].get_value();
      joints_state_.velocity[i] = state_interfaces_[state_interface_indices_[i][1]].get_value();
      joints_state_.effort[i] = state_interfaces_[state_interface_indices_[i][2]].get_value();
      joints_state_.temperature[i] = state_interfaces_[state_interface_indices_[i][3]].get_value();
      joints_state_.valid[i] = state_interfaces_[state_interface_indices_[i][4]].get_value();
    }
    calibrated_ = state_interfaces_[system_state_interface_indices_[0]].get_value() > 0.5;
    communication_ok_ = state_interfaces_[system_state_interface_indices_[1]].get_value() > 0.5;
    physical_estop_ = state_interfaces_[system_state_interface_indices_[2]].get_value() > 0.5;
    io_period_ms_ = state_interfaces_[system_state_interface_indices_[3]].get_value();
    io_timeout_count_ = state_interfaces_[system_state_interface_indices_[4]].get_value();
    consecutive_io_failures_ =
        communication_ok_ ? 0 : consecutive_io_failures_ + 1;
  } else {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      joints_state_.position[i] = state_interfaces_[state_interface_indices_[i][0]].get_value();
      joints_state_.velocity[i] = state_interfaces_[state_interface_indices_[i][1]].get_value();
      joints_state_.effort[i] = state_interfaces_[state_interface_indices_[i][2]].get_value();
      // 仿真无电机温度或通信有效性通道；这些是占位状态，不是传感器测量。
      joints_state_.temperature[i] = 0.0;
      joints_state_.valid[i] = 1.0;
    }
    calibrated_ = true;
    communication_ok_ = true;
    physical_estop_ = false;
    consecutive_io_failures_ = 0;
  }
}

void NmpcWbcController::WriteHybridCommand(
    const HybridJointCommand& command) {
  if (IsRealHardware()) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      command_interfaces_[command_interface_indices_[i][0]].set_value(command.position[i]);
      command_interfaces_[command_interface_indices_[i][1]].set_value(command.velocity[i]);
      command_interfaces_[command_interface_indices_[i][2]].set_value(command.effort[i]);
      command_interfaces_[command_interface_indices_[i][3]].set_value(command.kp[i]);
      command_interfaces_[command_interface_indices_[i][4]].set_value(command.kd[i]);
    }
    double calibration = mode_ == OperatingMode::CALIBRATION ? 1.0 : 0.0;
    // 标定命令约定：1 请求标定，0 不请求，-1 清除旧标定；复位只发送一个周期。
    if (reset_calibration_pending_) {
      calibration = -1.0;
      reset_calibration_pending_ = false;
    }
    command_interfaces_[system_command_interface_indices_[0]].set_value(calibration);
    command_interfaces_[system_command_interface_indices_[1]].set_value(
        mode_ == OperatingMode::FAULT ? 1.0 : 0.0);
  } else {
    const auto& effort_limits = safety_monitor_->limits().effort_limit;
    // 仿真与真机保持同一 tau_ff + kp*(q_des-q) + kd*(dq_des-dq) 语义，
    // 合成后再按 URDF 力矩上限截断，避免把 PD 项遗漏在限幅之外。
    for (std::size_t i = 0; i < kJointCount; ++i) {
      command_interfaces_[command_interface_indices_[i][0]].set_value(
          BoundedEquivalentEffort(command, joints_state_, i, effort_limits[i]));
    }
  }
}

void NmpcWbcController::WriteSafeCommand(bool reset_calibration) {
  // 默认零前馈、零位置刚度，仅施加阻尼；只有仿真 PASSIVE 可选固定趴姿保持。
  // FAULT 不使用该姿态保持分支，且会通过 WriteHybridCommand 向真机传递急停状态。
  HybridJointCommand command;
  const bool hold_simulation_pose =
      !IsRealHardware() && mode_ == OperatingMode::PASSIVE &&
      simulation_passive_hold_;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (hold_simulation_pose) {
      command.position[i] = passive_joint_positions_[i];
      command.kp[i] =
          (i % 3 == 0) ? simulation_passive_hip_kp_
                       : simulation_passive_leg_kp_;
      command.kd[i] = simulation_passive_kd_;
    } else {
      command.position[i] = std::isfinite(joints_state_.position[i])
                                ? joints_state_.position[i]
                                : 0.0;
      command.kd[i] = safe_damping_;
    }
  }
  // 合并请求而非覆盖，保留尚未发送到硬件的标定复位脉冲。
  reset_calibration_pending_ = reset_calibration_pending_ || reset_calibration;
  WriteHybridCommand(command);
}

}  // namespace custom_dog_control
