#include "custom_dog_control/model/RobotModelConfig.hpp"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace custom_dog_control {
namespace {
void Check(double value, double low, double high, const char *name) {
  if (!std::isfinite(value) || value < low || value > high)
    throw std::invalid_argument(std::string("invalid model parameter: ") +
                                name);
}
} // namespace
void RobotModelConfig::Validate() const {
#define QR_PARAM(name, value, lower, upper) Check(name, lower, upper, #name);
#include "custom_dog_control/model/RobotModelConfig.def"
#undef QR_PARAM
}
RobotModelConfig RobotModelConfig::Load(const std::string &path) {
  const auto root = YAML::LoadFile(path);
  const auto node = root["model"];
  if (!node || !node.IsMap() || root.size() != 1)
    throw std::invalid_argument("invalid model configuration");
  RobotModelConfig out;
  size_t count = 0;
#define QR_PARAM(name, value, lower, upper)                                    \
  out.name = node[#name].as<double>();                                         \
  ++count;
#include "custom_dog_control/model/RobotModelConfig.def"
#undef QR_PARAM
  if (node.size() != count)
    throw std::invalid_argument("unknown or duplicate model parameter");
  out.Validate();
  return out;
}

} // namespace custom_dog_control
