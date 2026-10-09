#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
namespace custom_dog_control {
namespace {
void Check(double value, double low, double high, const char *name) {
  if (!std::isfinite(value) || value < low || value > high)
    throw std::invalid_argument(std::string("invalid parameter: ") + name);
}
} // namespace

void PrecisionConfig::Validate() const {
#define QR_PARAM(name, value, lower, upper) Check(name, lower, upper, #name);
#include "custom_dog_control/precision/PrecisionConfig.def"
#undef QR_PARAM
  if (load_off_n >= load_on_n || candidate_force_n >= load_on_n ||
      candidate_force_n >= probe_force_n ||
      startup_settle_s >= startup_timeout_s ||
      startup_grace_s >= startup_settle_s ||
      confirm_dwell_s + candidate_dwell_s >= touchdown_timeout_s ||
      early_contact_delay_s >= swing_s * early_contact_fraction ||
      slip_speed_mps >= contact_speed_mps)
    throw std::invalid_argument("inconsistent precision thresholds/timing");
}
PrecisionConfig PrecisionConfig::Load(const std::string &path) {
  const auto root = YAML::LoadFile(path);
  const auto node = root["control"];
  if (!node || !node.IsMap() || root.size() != 1)
    throw std::invalid_argument("invalid control configuration");
  PrecisionConfig out;
  size_t count = 0;
#define QR_PARAM(name, value, lower, upper)                                    \
  out.name = node[#name].as<double>();                                         \
  ++count;
#include "custom_dog_control/precision/PrecisionConfig.def"
#undef QR_PARAM
  if (node.size() != count)
    throw std::invalid_argument("unknown or duplicate control parameter");
  out.Validate();
  return out;
}
} // namespace custom_dog_control
