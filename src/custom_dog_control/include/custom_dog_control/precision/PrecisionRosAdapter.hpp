#pragma once
#include "custom_dog_control/precision/PrecisionChannel.hpp"
#include "custom_dog_control/precision/PrecisionPlanner.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
namespace custom_dog_control {
class PrecisionRosAdapter {
public:
  PrecisionRosAdapter(rclcpp_lifecycle::LifecycleNode::SharedPtr,
                      PrecisionPlanner &, PrecisionChannel &,
                      const PrecisionConfig &);
  ~PrecisionRosAdapter();
  void Activate();
  void Deactivate();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace custom_dog_control
