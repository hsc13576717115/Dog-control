#pragma once
#include "custom_dog_control/model/RobotModelConfig.hpp"
#include "custom_dog_control/nmpc/KinematicStateEstimator.hpp"
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include <memory>
namespace custom_dog_control {
class PrecisionModel {
public:
  PrecisionModel(const ocs2::PinocchioInterface &,
                 const ocs2::CentroidalModelInfo &, const std::string &urdf,
                 RobotModelConfig model_config = {},
                 PrecisionConfig config = {});
  ~PrecisionModel();
  void Reset();
  void Measure(const JointSample &, const EstimatedState &, double dt);
  const std::array<Eigen::Vector3d, 4> &feet() const;
  const std::array<Eigen::Vector3d, 4> &velocities() const;
  const std::array<Eigen::Vector3d, 4> &forces() const;
  bool Inverse(const WholeBodyReference &, const JointSample &,
               Eigen::VectorXd &q, Eigen::VectorXd &dq);
  Eigen::Vector3d CenterOfMass(const Eigen::VectorXd &q);
  bool CollisionFree(const Eigen::VectorXd &, const Eigen::Vector3d &pad_center,
                     double pad_height, const Eigen::Vector2d &size);
  Eigen::VectorXd Rbd(const JointSample &, const EstimatedState &) const;
  int slot(size_t i) const;
  bool ApplyProbe(int foot, double force, HybridJointCommand &command);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace custom_dog_control
