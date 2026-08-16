#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

Eigen::Vector3d EulerZyxFromRotation(const Eigen::Matrix3d& rotation);

struct EstimatedState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d euler_zyx = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_world = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, kLegCount> foot_position_world{};
  double stamp_seconds = 0.0;
  bool valid = false;
};

struct EstimatorNoise {
  double imu_position = 0.02;
  double imu_velocity = 0.02;
  double foot_process_position = 0.002;
  double foot_sensor_position = 0.005;
  double foot_sensor_velocity = 0.10;
  double foot_height = 0.01;
  double swing_covariance_scale = 100.0;
};

class KinematicStateEstimator {
 public:
  KinematicStateEstimator(
      const ocs2::PinocchioInterface& interface,
      ocs2::CentroidalModelInfo info,
      EstimatorNoise noise = {});

  EstimatedState Update(
      const JointSample& joints,
      const ImuSample& imu,
      const std::array<bool, kLegCount>& planned_contacts,
      double dt);

  void Reset(double nominal_height_m);

 private:
  using StateVector = Eigen::Matrix<double, 18, 1>;
  using StateMatrix = Eigen::Matrix<double, 18, 18>;
  using ObservationVector = Eigen::Matrix<double, 28, 1>;
  using ObservationMatrix = Eigen::Matrix<double, 28, 18>;
  using ObservationCovariance = Eigen::Matrix<double, 28, 28>;

  ocs2::PinocchioInterface pinocchio_interface_;
  ocs2::CentroidalModelInfo info_;
  EstimatorNoise noise_;
  std::array<int, kJointCount> model_joint_slots_{};

  StateVector state_ = StateVector::Zero();
  StateMatrix covariance_ = StateMatrix::Identity() * 100.0;
  ObservationMatrix observation_model_ = ObservationMatrix::Zero();
  bool initialized_ = false;
};

}  // namespace custom_dog_control
