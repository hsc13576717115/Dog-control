#pragma once

#include <algorithm>
#include <cmath>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

// 四腿复用 hip/thigh/calf 三个刚度；默认零目标速度、零前馈力矩。
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

// Last simulation output boundary: clamp the combined PD/feedforward effort.
// A zero gain does not suppress NaN arithmetic (0 * NaN is NaN), so an invalid
// sample must produce a finite disabled output instead of poisoning physics.
inline double BoundedEquivalentEffort(
    const HybridJointCommand& command, const JointSample& measured,
    std::size_t joint, double effort_limit) {
  const double effort = EquivalentEffort(command, measured, joint);
  if (!std::isfinite(effort) || !std::isfinite(effort_limit) || effort_limit < 0.0) {
    return 0.0;
  }
  return std::clamp(effort, -effort_limit, effort_limit);
}

// 同时插值增益和目标会在 PD 力矩中产生交叉项，故用前馈补偿，保证在
// 当前 measured 下总力矩等于两端总力矩的线性插值。调用方提供 [0,1] 的 alpha；
// 此处不限幅，后续输出饱和以及下一周期测量变化都会影响实际力矩。
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
