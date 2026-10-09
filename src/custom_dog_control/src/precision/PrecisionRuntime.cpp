#include "custom_dog_control/precision/PrecisionRuntime.hpp"
#include "custom_dog_control/precision/PrecisionExecutionCore.hpp"
#include "custom_dog_control/precision/PrecisionRosAdapter.hpp"
namespace custom_dog_control {
struct PrecisionRuntime::Impl {
  PrecisionChannel channel;
  PrecisionPlanner planner;
  PrecisionExecutionCore core;
  PrecisionRosAdapter ros;
  Impl(rclcpp_lifecycle::LifecycleNode::SharedPtr n, const RobotModel &r,
       const std::string &u, const std::string &t, const RobotModelConfig &m,
       const PrecisionConfig &c)
      : planner(r, u, t, m, c), core(r, u, t, m, c),
        ros(n, planner, channel, c) {}
};
PrecisionRuntime::PrecisionRuntime(rclcpp_lifecycle::LifecycleNode::SharedPtr n,
                                   const RobotModel &r, const std::string &u,
                                   const std::string &t,
                                   const RobotModelConfig &m,
                                   const PrecisionConfig &c)
    : impl_(std::make_unique<Impl>(n, r, u, t, m, c)) {}
PrecisionRuntime::~PrecisionRuntime() = default;
void PrecisionRuntime::Activate() {
  // ros2_control lifecycle guarantees update is inactive here.
  impl_->core.Reset();
  impl_->ros.Activate();
}
void PrecisionRuntime::Deactivate() { impl_->ros.Deactivate(); }
bool PrecisionRuntime::Update(double now, double dt, const JointSample &j,
                              const ImuSample &i, bool stop,
                              HybridJointCommand &out) {
  if (!impl_->channel.active)
    return false;
  const auto command = *impl_->channel.pending.readFromRT();
  const bool valid = impl_->core.Update(now, dt, j, i, stop, command,
                                        impl_->channel.cancel, out);
  impl_->channel.snapshot.Publish(impl_->core.snapshot());
  return valid;
}
} // namespace custom_dog_control
