#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"

#include <algorithm>
#include <cmath>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace custom_dog_control {
namespace {

Eigen::Quaterniond QuaternionFrom(const ImuSample& imu) {
  Eigen::Quaterniond quaternion(
      imu.orientation_wxyz[0], imu.orientation_wxyz[1],
      imu.orientation_wxyz[2], imu.orientation_wxyz[3]);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-6) {
    return Eigen::Quaterniond::Identity();
  }
  return quaternion.normalized();
}

}  // namespace

Eigen::Vector3d EulerZyxFromRotation(const Eigen::Matrix3d& rotation) {
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  return {yaw, pitch, roll};
}

KinematicStateEstimator::KinematicStateEstimator(
    const ocs2::PinocchioInterface& interface,
    ocs2::CentroidalModelInfo info,
    EstimatorNoise noise)
    : pinocchio_interface_(interface), info_(std::move(info)), noise_(noise) {
  const auto& model = pinocchio_interface_.getModel();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto joint_id = model.getJointId(std::string(kJointNames[i]));
    model_joint_slots_[i] = model.joints[joint_id].idx_q() - 6;
  }
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    observation_model_.block<3, 3>(3 * leg, 0) = Eigen::Matrix3d::Identity();
    observation_model_.block<3, 3>(3 * leg, 6 + 3 * leg) = -Eigen::Matrix3d::Identity();
    observation_model_.block<3, 3>(12 + 3 * leg, 3) = Eigen::Matrix3d::Identity();
    observation_model_(24 + leg, 6 + 3 * leg + 2) = 1.0;
  }
}

void KinematicStateEstimator::Reset(double nominal_height_m) {
  state_.setZero();
  state_(2) = nominal_height_m;
  covariance_.setIdentity();
  covariance_ *= 100.0;
  initialized_ = false;
}

EstimatedState KinematicStateEstimator::Update(
    const JointSample& joints,
    const ImuSample& imu,
    const std::array<bool, kLegCount>& planned_contacts,
    double dt) {
  EstimatedState output;
  if (!imu.valid || dt <= 0.0 || dt > 0.1) {
    return output;
  }

  const Eigen::Quaterniond orientation = QuaternionFrom(imu);
  const Eigen::Matrix3d rotation_world_from_body = orientation.toRotationMatrix();
  const Eigen::Vector3d euler_zyx =
      EulerZyxFromRotation(rotation_world_from_body);
  const Eigen::Vector3d angular_body(
      imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]);
  const Eigen::Vector3d angular_world = rotation_world_from_body * angular_body;

  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  ocs2::vector_t q = ocs2::vector_t::Zero(info_.generalizedCoordinatesNum);
  ocs2::vector_t v = ocs2::vector_t::Zero(info_.generalizedCoordinatesNum);
  q.segment<3>(3) = euler_zyx;
  v.segment<3>(3) =
      ocs2::getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<double>(
          euler_zyx, angular_world);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    q(6 + model_joint_slots_[i]) = joints.position[i];
    v(6 + model_joint_slots_[i]) = joints.velocity[i];
  }

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);

  std::array<Eigen::Vector3d, kLegCount> foot_in_world_at_origin{};
  std::array<Eigen::Vector3d, kLegCount> foot_velocity_at_origin{};
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    const auto frame_id = model.getFrameId(std::string(kFootFrameNames[leg]));
    foot_in_world_at_origin[leg] = data.oMf[frame_id].translation();
    foot_velocity_at_origin[leg] =
        pinocchio::getFrameVelocity(
            model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();
  }

  if (!initialized_) {
    double height_sum = 0.0;
    int contact_count = 0;
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      if (planned_contacts[leg]) {
        height_sum += -foot_in_world_at_origin[leg].z();
        ++contact_count;
      }
    }
    state_(2) = contact_count > 0 ? height_sum / contact_count : 0.28;
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
      state_.segment<3>(6 + 3 * leg) =
          state_.head<3>() + foot_in_world_at_origin[leg];
      if (planned_contacts[leg]) {
        state_(6 + 3 * leg + 2) = 0.0;
      }
    }
    initialized_ = true;
  }

  StateMatrix transition = StateMatrix::Identity();
  transition.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 18, 3> input = Eigen::Matrix<double, 18, 3>::Zero();
  input.block<3, 3>(0, 0) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
  input.block<3, 3>(3, 0) = dt * Eigen::Matrix3d::Identity();

  StateMatrix process_noise = StateMatrix::Zero();
  process_noise.block<3, 3>(0, 0) =
      noise_.imu_position * dt / 20.0 * Eigen::Matrix3d::Identity();
  process_noise.block<3, 3>(3, 3) =
      noise_.imu_velocity * dt * 9.81 / 20.0 * Eigen::Matrix3d::Identity();
  process_noise.block<12, 12>(6, 6) =
      noise_.foot_process_position * dt *
      Eigen::Matrix<double, 12, 12>::Identity();

  ObservationCovariance measurement_noise = ObservationCovariance::Zero();
  ObservationVector measurement = ObservationVector::Zero();
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    const double scale = planned_contacts[leg] ? 1.0 : noise_.swing_covariance_scale;
    process_noise.block<3, 3>(6 + 3 * leg, 6 + 3 * leg) *= scale;
    measurement_noise.block<3, 3>(3 * leg, 3 * leg) =
        scale * noise_.foot_sensor_position * Eigen::Matrix3d::Identity();
    measurement_noise.block<3, 3>(12 + 3 * leg, 12 + 3 * leg) =
        scale * noise_.foot_sensor_velocity * Eigen::Matrix3d::Identity();
    measurement_noise(24 + leg, 24 + leg) =
        scale * noise_.foot_height;
    measurement.segment<3>(3 * leg) = -foot_in_world_at_origin[leg];
    measurement.segment<3>(12 + 3 * leg) = -foot_velocity_at_origin[leg];
    measurement(24 + leg) = 0.0;
  }

  const Eigen::Vector3d acceleration_body(
      imu.linear_acceleration[0], imu.linear_acceleration[1],
      imu.linear_acceleration[2]);
  const Eigen::Vector3d acceleration_world =
      rotation_world_from_body * acceleration_body + Eigen::Vector3d(0.0, 0.0, -9.81);

  const StateVector predicted_state = transition * state_ + input * acceleration_world;
  const StateMatrix predicted_covariance =
      transition * covariance_ * transition.transpose() + process_noise;
  const ObservationCovariance innovation_covariance =
      observation_model_ * predicted_covariance * observation_model_.transpose() +
      measurement_noise;
  const auto kalman_gain =
      predicted_covariance * observation_model_.transpose() *
      innovation_covariance.ldlt().solve(ObservationCovariance::Identity());
  state_ = predicted_state +
           kalman_gain * (measurement - observation_model_ * predicted_state);
  covariance_ =
      (StateMatrix::Identity() - kalman_gain * observation_model_) *
      predicted_covariance;
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());

  output.position = state_.head<3>();
  output.velocity_world = state_.segment<3>(3);
  output.euler_zyx = euler_zyx;
  output.angular_velocity_world = angular_world;
  output.stamp_seconds = imu.stamp_seconds;
  for (std::size_t leg = 0; leg < kLegCount; ++leg) {
    output.foot_position_world[leg] = state_.segment<3>(6 + 3 * leg);
  }
  output.valid = state_.allFinite() && covariance_.allFinite();
  return output;
}

}  // namespace custom_dog_control
