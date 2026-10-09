#pragma once
#include "custom_dog_control/control/ControlTypes.hpp"
#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
namespace custom_dog_control {
// Orchestrates independent planning, execution and ROS adapters. Simulation
// only.
class PrecisionRuntime {
public:
  PrecisionRuntime(rclcpp_lifecycle::LifecycleNode::SharedPtr,
                   const RobotModel &, const std::string &urdf,
                   const std::string &task, const RobotModelConfig &,
                   const PrecisionConfig &);
  ~PrecisionRuntime();
  void Activate();
  void Deactivate();
  bool Update(double now, double dt, const JointSample &, const ImuSample &,
              bool estop, HybridJointCommand &);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace custom_dog_control
