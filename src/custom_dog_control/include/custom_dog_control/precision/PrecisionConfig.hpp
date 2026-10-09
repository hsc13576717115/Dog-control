#pragma once
#include <string>
namespace custom_dog_control {
struct PrecisionConfig {
#define QR_PARAM(name, value, lower, upper) double name = value;
#include "custom_dog_control/precision/PrecisionConfig.def"
#undef QR_PARAM
  void Validate() const;
  static PrecisionConfig Load(const std::string &path);
};
} // namespace custom_dog_control
