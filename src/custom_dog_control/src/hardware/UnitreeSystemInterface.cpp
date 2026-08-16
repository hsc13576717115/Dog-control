#include "custom_dog_control/hardware/UnitreeSystemInterface.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace custom_dog_control {
namespace {

bool ParseBool(const std::string& value) {
  return value == "true" || value == "True" || value == "1";
}

template <typename T>
T ParameterOr(
    const hardware_interface::HardwareInfo& info,
    const std::string& name,
    const T& default_value);

template <>
double ParameterOr(
    const hardware_interface::HardwareInfo& info,
    const std::string& name,
    const double& default_value) {
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end()
             ? default_value
             : std::stod(iterator->second);
}

template <>
std::string ParameterOr(
    const hardware_interface::HardwareInfo& info,
    const std::string& name,
    const std::string& default_value) {
  const auto iterator = info.hardware_parameters.find(name);
  return iterator == info.hardware_parameters.end()
             ? default_value
             : iterator->second;
}

}  // namespace

hardware_interface::CallbackReturn UnitreeSystemInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(rclcpp::get_logger("UnitreeSystemInterface"), "Expected 12 joints");
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (info_.joints[i].name != kJointNames[i]) {
      RCLCPP_ERROR(
          rclcpp::get_logger("UnitreeSystemInterface"),
          "Joint order mismatch at index %zu: expected %s, got %s",
          i, kJointNames[i].data(), info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    state_.valid[i] = 0.0;
  }
  return ParseHardwareParameters()
             ? hardware_interface::CallbackReturn::SUCCESS
             : hardware_interface::CallbackReturn::ERROR;
}

bool UnitreeSystemInterface::ParseHardwareParameters() {
  try {
    drive_parameters_.serial_ports = {
        ParameterOr<std::string>(info_, "fr_port", "/dev/ttyS3"),
        ParameterOr<std::string>(info_, "fl_port", "/dev/ttyS4"),
        ParameterOr<std::string>(info_, "rr_port", "/dev/ttyS7"),
        ParameterOr<std::string>(info_, "rl_port", "/dev/ttyS8")};
    drive_parameters_.base_gear_ratio =
        ParameterOr<double>(info_, "base_gear_ratio", 6.33);
    drive_parameters_.calf_total_gear_ratio =
        ParameterOr<double>(info_, "calf_gear_ratio", 12.66);
    drive_parameters_.use_parallel_leg_io = ParseBool(
        ParameterOr<std::string>(info_, "parallel_leg_io", "true"));
    physical_estop_verified_ = ParseBool(
        ParameterOr<std::string>(info_, "physical_estop_verified", "false"));
    // Verification is an activation interlock, not an asserted E-stop signal.
    // The physical E-stop remains independent and cuts actuator power directly.
    physical_estop_ = 0.0;
    safe_damping_ = ParameterOr<double>(info_, "safe_damping", 1.0);
    drive_parameters_.calibration_pose = {
        ParameterOr<double>(
            info_, "calibration_hip", drive_parameters_.calibration_pose[0]),
        ParameterOr<double>(
            info_, "calibration_thigh", drive_parameters_.calibration_pose[1]),
        ParameterOr<double>(
            info_, "calibration_calf", drive_parameters_.calibration_pose[2])};

    std::stringstream directions(
        ParameterOr<std::string>(
            info_, "motor_directions",
            "1 1 1 1 1 1 1 1 1 1 1 1"));
    for (auto& direction : drive_parameters_.motor_directions) {
      if (!(directions >> direction) ||
          (direction != 1.0 && direction != -1.0)) {
        throw std::invalid_argument("motor_directions must contain twelve +/-1 values");
      }
    }
    return true;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Invalid hardware parameters: %s", exception.what());
    return false;
  }
}

std::vector<hardware_interface::StateInterface>
UnitreeSystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 5 + 5);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string name(kJointNames[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &state_.position[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &state_.velocity[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &state_.effort[i]);
    interfaces.emplace_back(name, "temperature", &state_.temperature[i]);
    interfaces.emplace_back(name, "valid", &state_.valid[i]);
  }
  interfaces.emplace_back("custom_dog", "calibrated", &calibrated_);
  interfaces.emplace_back("custom_dog", "communication_ok", &communication_ok_);
  interfaces.emplace_back("custom_dog", "physical_estop", &physical_estop_);
  interfaces.emplace_back("custom_dog", "io_period_ms", &io_period_ms_);
  interfaces.emplace_back("custom_dog", "io_timeout_count", &io_timeout_count_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
UnitreeSystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount * 5 + 2);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string name(kJointNames[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &command_.position[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &command_.velocity[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &command_.effort[i]);
    interfaces.emplace_back(name, "kp", &command_.kp[i]);
    interfaces.emplace_back(name, "kd", &command_.kd[i]);
  }
  interfaces.emplace_back("custom_dog", "calibrate", &calibration_request_);
  interfaces.emplace_back("custom_dog", "emergency_stop", &emergency_stop_request_);
  return interfaces;
}

hardware_interface::CallbackReturn UnitreeSystemInterface::on_configure(
    const rclcpp_lifecycle::State&) {
  try {
    sdk_ = std::make_unique<IOSDK>(drive_parameters_);
    SetDampingCommand();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Failed to configure actuator SDK: %s", exception.what());
    sdk_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn UnitreeSystemInterface::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!physical_estop_verified_) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Activation blocked: physical_estop_verified is false");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!sdk_) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    sdk_->Activate();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(
        rclcpp::get_logger("UnitreeSystemInterface"),
        "Failed to activate actuator SDK: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn UnitreeSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State&) {
  emergency_stop_request_ = 1.0;
  SetDampingCommand();
  FillLowCommand();
  if (sdk_) {
    sdk_->SendReceive(low_command_, low_state_, false);
    sdk_->Deactivate();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UnitreeSystemInterface::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!sdk_) {
    return hardware_interface::return_type::ERROR;
  }
  if (calibration_request_ < -0.5) {
    sdk_->ResetCalibration();
  }
  const auto start = std::chrono::steady_clock::now();
  sdk_->SendReceive(low_command_, low_state_, calibration_request_ > 0.5);
  io_period_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

  for (std::size_t i = 0; i < kJointCount; ++i) {
    state_.position[i] = low_state_.motors[i].position;
    state_.velocity[i] = low_state_.motors[i].velocity;
    state_.effort[i] = low_state_.motors[i].effort;
    state_.temperature[i] = low_state_.motors[i].temperature;
    state_.valid[i] = low_state_.motors[i].fault == 0 ? 1.0 : 0.0;
  }
  calibrated_ = sdk_->IsCalibrated() ? 1.0 : 0.0;
  communication_ok_ = sdk_->CommunicationOk() ? 1.0 : 0.0;
  io_timeout_count_ = static_cast<double>(sdk_->TotalFailures());
  consecutive_failures_ =
      sdk_->CommunicationOk() ? 0 : consecutive_failures_ + 1;
  return consecutive_failures_ >= 3
             ? hardware_interface::return_type::ERROR
             : hardware_interface::return_type::OK;
}

void UnitreeSystemInterface::SetDampingCommand() {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command_.position[i] = state_.position[i];
    command_.velocity[i] = 0.0;
    command_.effort[i] = 0.0;
    command_.kp[i] = 0.0;
    command_.kd[i] = safe_damping_;
  }
}

void UnitreeSystemInterface::FillLowCommand() {
  for (std::size_t i = 0; i < kJointCount; ++i) {
    auto& motor = low_command_.motors[i];
    motor.mode = static_cast<std::uint32_t>(ActuatorControlMode::COMPOUND);
    const double fallback_position =
        std::isfinite(state_.position[i]) ? state_.position[i] : 0.0;
    motor.position = static_cast<float>(
        std::isfinite(command_.position[i]) ? command_.position[i] : fallback_position);
    motor.velocity = static_cast<float>(
        std::isfinite(command_.velocity[i]) ? command_.velocity[i] : 0.0);
    motor.effort = static_cast<float>(
        std::isfinite(command_.effort[i]) ? command_.effort[i] : 0.0);
    motor.kp = static_cast<float>(
        std::isfinite(command_.kp[i]) ? std::max(0.0, command_.kp[i]) : 0.0);
    motor.kd = static_cast<float>(
        std::isfinite(command_.kd[i]) ? std::max(0.0, command_.kd[i]) : safe_damping_);
  }
}

hardware_interface::return_type UnitreeSystemInterface::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (emergency_stop_request_ > 0.5) {
    SetDampingCommand();
  }
  FillLowCommand();
  return hardware_interface::return_type::OK;
}

}  // namespace custom_dog_control

PLUGINLIB_EXPORT_CLASS(
    custom_dog_control::UnitreeSystemInterface,
    hardware_interface::SystemInterface)
