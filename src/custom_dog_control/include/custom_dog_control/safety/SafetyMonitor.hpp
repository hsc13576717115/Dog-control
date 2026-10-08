#pragma once

#include <array>
#include <string>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

struct SafetyLimits {
  double imu_timeout_s = 0.10;
  double max_temperature_c = 80.0;
  double max_roll_pitch_rad = 0.75;
  double max_policy_age_s = 0.10;
  double joint_position_tolerance_rad = 0.005;
  int max_consecutive_io_failures = 3;
  std::array<double, kJointCount> lower_position{};
  std::array<double, kJointCount> upper_position{};
  std::array<double, kJointCount> effort_limit{};
};

// 每个控制周期的一致快照；now_seconds 与 imu.stamp_seconds 必须同源，
// policy_age_s 则是后端已经算好的时长。dynamic_mode 控制策略时效检查，
// check_joint_limits 允许在尚未完成编码器零点标定时跳过位置范围检查。
struct SafetyInput {
  JointSample joints;
  ImuSample imu;
  double now_seconds = 0.0;
  double policy_age_s = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  int consecutive_io_failures = 0;
  bool communication_ok = true;
  bool physical_estop = false;
  bool software_estop = false;
  bool solver_valid = true;
  bool dynamic_mode = false;
  bool check_joint_limits = true;
};

// 故障锁存器，由控制线程串行使用。Evaluate 只判定并记录首个故障，
// 不写电机命令；调用方负责进入 FAULT 和下发安全输出。
class SafetyMonitor {
 public:
  explicit SafetyMonitor(SafetyLimits limits);

  bool Evaluate(const SafetyInput& input);
  // 显式清除锁存；能否重新使能、是否需要重标定由外层状态机决定。
  void Reset();
  bool faulted() const { return faulted_; }
  const std::string& reason() const { return reason_; }
  const SafetyLimits& limits() const { return limits_; }

 private:
  bool Trip(const std::string& reason);

  SafetyLimits limits_;
  bool faulted_ = false;
  std::string reason_;
};

}  // namespace custom_dog_control
