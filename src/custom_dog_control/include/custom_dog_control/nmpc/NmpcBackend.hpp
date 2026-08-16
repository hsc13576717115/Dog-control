#pragma once

#include <array>
#include <memory>
#include <string>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/nmpc/ModelValidator.hpp"

namespace custom_dog_control {

struct NmpcBackendConfig {
  std::string task_file;
  std::string reference_file;
  std::string urdf_file;
  // Matches legged_control's mpcDesiredFrequency. WBC runs in the
  // ros2_control update loop, configured independently at 1 kHz.
  double frequency_hz = 50.0;
  double target_horizon_s = 1.0;
  double nominal_height_m = 0.28;
  double max_hip_angle_rad = 0.20;
  double joint_limit_margin_rad = 0.03;
  std::array<double, kJointCount> nominal_joint_positions{};
};

struct WbcOutput {
  HybridJointCommand command;
  double equality_residual = 0.0;
  double inequality_violation = 0.0;
  double solve_time_ms = 0.0;
  bool valid = false;
};

class NmpcBackend {
 public:
  NmpcBackend();
  ~NmpcBackend();

  NmpcBackend(const NmpcBackend&) = delete;
  NmpcBackend& operator=(const NmpcBackend&) = delete;

  ModelValidationResult Configure(const NmpcBackendConfig& config);
  void Start();
  void Stop();

  ocs2::vector_t UpdateObservation(
      const EstimatedState& estimate,
      const JointSample& joints,
      double observation_time,
      std::size_t planned_mode);
  void SetVelocityCommand(
      const VelocityCommand& command, bool reanchor_position = false,
      bool reanchor_heading = false);
  void RequestGait(bool trot);
  bool EvaluatePolicy(double now_seconds, PolicySample& output);
  WbcOutput ComputeWbc(
      const PolicySample& policy,
      const ocs2::vector_t& measured_rbd_state,
      double period_seconds);

  bool configured() const;
  bool solverHealthy() const;
  double lastSolveTimeMs() const;
  double policyAgeSeconds(double now_seconds) const;
  std::string lastError() const;

  const ocs2::PinocchioInterface& pinocchioInterface() const;
  const ocs2::CentroidalModelInfo& modelInfo() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace custom_dog_control
