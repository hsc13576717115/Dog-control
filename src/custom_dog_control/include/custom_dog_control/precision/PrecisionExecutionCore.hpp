#pragma once
#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionState.hpp"
#include <memory>
namespace custom_dog_control {
// No ROS calls, callbacks, or shared model caches. One update-thread owner.
class PrecisionExecutionCore {
public:
  PrecisionExecutionCore(const RobotModel &, const std::string &,
                         const std::string &, const RobotModelConfig &,
                         const PrecisionConfig &);
  ~PrecisionExecutionCore();
  void Reset(); // Only while update is inactive.
  bool Update(double now, double dt, const JointSample &, const ImuSample &,
              bool estop, const PrecisionCommand &, bool cancel,
              HybridJointCommand &);
  const PrecisionSnapshot &snapshot() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace custom_dog_control
