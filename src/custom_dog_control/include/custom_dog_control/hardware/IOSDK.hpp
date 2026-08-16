#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "custom_dog_control/hardware/ActuatorTypes.hpp"
#include "custom_dog_control/hardware/Calibration.hpp"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace custom_dog_control {

// GO-M8010-6 transport and output-side coordinate conversion. This class has
// no robot kinematics; all positions exposed here are direct URDF coordinates.
class IOSDK final {
 public:
  explicit IOSDK(const DriveParameters& drive_parameters);
  ~IOSDK();

  IOSDK(const IOSDK&) = delete;
  IOSDK& operator=(const IOSDK&) = delete;

  void SendReceive(
      const LowLevelCommand& command, LowLevelState& state,
      bool calibration_requested);
  void Activate();
  void Deactivate();
  void ResetCalibration() { calibrated_ = false; }
  bool IsCalibrated() const { return calibrated_; }
  bool CommunicationOk() const { return last_cycle_failures_.load() == 0; }
  int LastCycleFailures() const { return last_cycle_failures_.load(); }
  std::uint64_t TotalFailures() const { return total_failures_.load(); }
  const std::array<float, kJointCount>& CalibrationOffsets() const {
    return calibration_offsets_;
  }

 private:
  void OpenSerialPorts();
  void InitializeMotorMetadata();
  void StartWorkers();
  void StopWorkers();
  void WorkerLoop(int leg);
  void TryCalibrate(bool calibration_requested);
  void FinalizeCycleCalibration(
      bool calibration_requested, LowLevelState& state);
  void CalibrateLeg(int leg);
  void SendDirectLegCommand(
      int leg, const std::array<ActuatorCommand, kJointsPerLeg>& commands);
  std::array<ActuatorCommand, kJointsPerLeg> BuildZeroTorqueCommand() const;
  void SendReceiveLeg(int leg, const LowLevelCommand& command, LowLevelState& state);
  void PopulateMotorCommand(int leg, int joint, const ActuatorCommand& command);
  void UpdateMotorStateFromFeedback(int leg, int joint, LowLevelState& state);
  void MarkMotorOffline(int leg, int joint, LowLevelState& state) const;
  float GearRatioForJoint(int joint) const;

  std::vector<SerialPort*> serials_;
  std::array<std::string, kLegCount> serial_ports_;
  std::array<MotorCmd, kJointCount> motor_commands_;
  std::array<MotorData, kJointCount> motor_data_;
  std::set<int> active_legs_;
  std::array<float, kJointCount> calibration_offsets_{};
  std::array<std::thread, kLegCount> workers_;
  std::mutex worker_mutex_;
  std::condition_variable dispatch_cv_;
  std::condition_variable completed_cv_;
  const LowLevelCommand* active_command_ = nullptr;
  LowLevelState* active_state_ = nullptr;
  std::size_t dispatch_epoch_ = 0;
  std::size_t completed_workers_ = 0;
  bool workers_stopping_ = false;
  bool calibrated_ = false;
  bool use_parallel_leg_io_ = false;
  bool active_ = false;
  mutable std::atomic<int> current_cycle_failures_{0};
  std::atomic<int> last_cycle_failures_{0};
  mutable std::atomic<std::uint64_t> total_failures_{0};
  float base_gear_ratio_ = 6.33F;
  float calf_total_gear_ratio_ = 12.66F;
  std::array<float, kJointCount> motor_directions_{};
  std::array<double, kJointsPerLeg> calibration_pose_{};
};

}  // namespace custom_dog_control
