#pragma once
#include <string>
namespace custom_dog_control {
struct RobotModelConfig {
#define QR_PARAM(name, value, lower, upper) double name = value;
#include "custom_dog_control/model/RobotModelConfig.def"
#undef QR_PARAM
  void Validate() const;
  static RobotModelConfig Load(const std::string &path);
};
} // namespace custom_dog_control
