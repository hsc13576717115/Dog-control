#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/hardware/ActuatorTypes.hpp"
#include "custom_dog_control/hardware/IOSDK.hpp"

namespace custom_dog_control {

class UnitreeSystemInterface final : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeSystemInterface)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  bool ParseHardwareParameters();
  void SetDampingCommand();
  void FillLowCommand();

  DriveParameters drive_parameters_;
  std::unique_ptr<IOSDK> sdk_;
  LowLevelCommand low_command_;
  LowLevelState low_state_;

  JointSample state_;
  HybridJointCommand command_;
  double calibration_request_ = 0.0;
  double emergency_stop_request_ = 0.0;
  double calibrated_ = 0.0;
  double communication_ok_ = 0.0;
  double physical_estop_ = 0.0;
  double io_period_ms_ = 0.0;
  double io_timeout_count_ = 0.0;
  double safe_damping_ = 1.0;
  bool physical_estop_verified_ = false;
  int consecutive_failures_ = 0;
};

}  // namespace custom_dog_control
