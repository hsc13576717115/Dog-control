#pragma once

#include <array>
#include <string>
#include <vector>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "custom_dog_control/control/ControlTypes.hpp"

namespace custom_dog_control {

// 配置阶段的模型检查结果。仅当 ok 为 true 时，调用方才能使用下面的关节映射。
struct ModelValidationResult {
  bool ok = false;
  double total_mass_kg = 0.0;
  // 下标采用硬件顺序 FR/FL/RR/RL，每腿 hip/thigh/calf。
  // 值是模型关节子向量中的槽位（idx_q/idx_v 减去 6），不是 Pinocchio joint_id。
  // 本项目使用六坐标浮动基座；不能直接套用 nq=7 的四元数 free-flyer 模型。
  // Validate 中未找到或索引/自由度不满足约定的关节保留 -1；成功时各槽位唯一。
  std::array<int, kJointCount> model_joint_slots{};
  std::vector<std::string> errors;

  // 汇总质量与所有已记录错误，供配置失败日志使用。
  std::string Summary() const;
};

class ModelValidator {
 public:
  // 核对质心状态维数、关节/足端名称、限位、质量和惯量，并在名义站姿下
  // 检查足端象限及 FK/Jacobian 的有限性。质量及容差单位均为 kg。
  // 会改写 interface.getData() 的运动学缓存，须在配置阶段串行调用；不修改模型。
  static ModelValidationResult Validate(
      ocs2::PinocchioInterface& interface,
      const ocs2::CentroidalModelInfo& info,
      double expected_mass_kg = 13.84916,
      double mass_tolerance_kg = 1e-5);
};

}  // namespace custom_dog_control
