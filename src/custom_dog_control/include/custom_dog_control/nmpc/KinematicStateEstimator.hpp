#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/precision/PrecisionTypes.hpp"

namespace custom_dog_control {

Eigen::Vector3d EulerZyxFromRotation(const Eigen::Matrix3d& rotation);

// 世界系采用 z 向上；位置/速度单位为 m、m/s，姿态按 yaw-pitch-roll 排列（rad）。
// 足端数组沿用 FR、FL、RR、RL 顺序；stamp_seconds 继承本次 IMU 时间戳。
struct EstimatedState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d euler_zyx = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_world = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, kLegCount> foot_position_world{};
  double stamp_seconds = 0.0;
  bool valid = false;
};

// 协方差的调节系数，直接用于构造 Q/R，并非传感器标准差。
// 摆动腿放大过程与测量协方差，减弱“足端静止且位于地面”的约束。
struct EstimatorNoise {
  double imu_position = 0.02;
  double imu_velocity = 0.02;
  double foot_process_position = 0.002;
  double foot_sensor_position = 0.005;
  double foot_sensor_velocity = 0.10;
  double foot_height = 0.01;
  double swing_covariance_scale = 100.0;
};

// 平地接触假设下的线性卡尔曼滤波器：估计基座平移和四足世界位置，
// 姿态直接来自 IMU，不估计 IMU 偏置。持有独立的 Pinocchio 数据缓存，
// 由控制线程串行调用；构造前应通过 ModelValidator 校验模型布局。
class KinematicStateEstimator {
 public:
  KinematicStateEstimator(
      const ocs2::PinocchioInterface& interface,
      ocs2::CentroidalModelInfo info,
      EstimatorNoise noise = {});

  // planned_contacts 是规划接触状态，不是触地传感器反馈。IMU 无效或
  // dt 不在 (0, 0.1] 秒时返回 valid=false；调用方仍需检查输出有效性。
  EstimatedState Update(
      const JointSample& joints,
      const ImuSample& imu,
      const std::array<bool, kLegCount>& planned_contacts,
      double dt);

  // Actual (or explicitly estimated) contact with independent height validity.
  // Unknown height is not zero; its observation row is removed.
  EstimatedState Update(const JointSample& joints, const ImuSample& imu,
                        const ContactSupport& support, double dt);

  // 清除滤波历史；下一次有效 Update 会用支撑腿运动学重新初始化高度。
  void Reset(double nominal_height_m);

 private:
  // x = [基座世界位置(3), 基座世界速度(3), 四足世界位置(12)]。
  // y = [基座相对足端位置(12), 四组基座速度(12), 四足高度(4)]。
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
