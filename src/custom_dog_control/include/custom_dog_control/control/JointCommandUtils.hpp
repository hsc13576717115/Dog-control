#pragma once

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

inline HybridJointCommand PositionPdCommand(
    const std::array<double, kJointCount>& position,
    const std::array<double, kJointsPerLeg>& stiffness, double damping) {
  HybridJointCommand command;
  command.position = position;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command.kp[i] = stiffness[i % kJointsPerLeg];
    command.kd[i] = damping;
  }
  return command;
}

inline double EquivalentEffort(
    const HybridJointCommand& command, const JointSample& measured,
    std::size_t joint) {
  return command.effort[joint] +
         command.kp[joint] * (command.position[joint] - measured.position[joint]) +
         command.kd[joint] * (command.velocity[joint] - measured.velocity[joint]);
}

// Interpolating gains and targets alone adds cross terms to the resulting
// torque. Compensate in feed-forward so the net effort remains a linear blend
// at the current measured state. The caller supplies alpha in [0, 1].
inline HybridJointCommand BlendHybridCommands(
    const HybridJointCommand& from, const HybridJointCommand& to,
    const JointSample& measured, double alpha) {
  HybridJointCommand command;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    command.position[i] = (1.0 - alpha) * from.position[i] + alpha * to.position[i];
    command.velocity[i] = (1.0 - alpha) * from.velocity[i] + alpha * to.velocity[i];
    command.kp[i] = (1.0 - alpha) * from.kp[i] + alpha * to.kp[i];
    command.kd[i] = (1.0 - alpha) * from.kd[i] + alpha * to.kd[i];
    const double net_effort =
        (1.0 - alpha) * EquivalentEffort(from, measured, i) +
        alpha * EquivalentEffort(to, measured, i);
    command.effort[i] = net_effort -
        command.kp[i] * (command.position[i] - measured.position[i]) -
        command.kd[i] * (command.velocity[i] - measured.velocity[i]);
  }
  return command;
}

}  // namespace custom_dog_control
