#pragma once
#include "custom_dog_control/nmpc/NmpcBackend.hpp"
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
namespace custom_dog_control {
// One controller, separate non-RT ROS callbacks and RT execution snapshot.
// The first release is intentionally simulation-only until effort calibration.
class PrecisionRuntime {
public:
  PrecisionRuntime(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                   const NmpcBackend &backend, const std::string &urdf,
                   const std::string &task);
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
