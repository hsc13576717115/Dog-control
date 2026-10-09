#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace qr_planning {
struct Step {
  uint8_t foot;
  uint32_t surface;
  std::array<double, 3> offset;
};
inline std::vector<Step> ParseFiniteTemplate(const YAML::Node &root) {
  if (!root.IsMap() || root.size() != 3)
    throw std::runtime_error("invalid_template_keys");
  if (root["frame"].as<std::string>() != "odom" ||
      root["target_mode"].as<std::string>() != "initial_foot_offset")
    throw std::runtime_error("unsupported_frame_or_target_mode");
  const auto list = root["steps"];
  if (!list.IsSequence() || list.size() == 0 || list.size() > 16)
    throw std::runtime_error("sequence_size_must_be_1_to_16");
  std::vector<Step> out;
  for (const auto &item : list) {
    if (!item.IsMap() || item.size() != 3)
      throw std::runtime_error("invalid_step_keys");
    const auto foot = item["foot"].as<int>();
    const auto surface = item["surface_id"].as<int>();
    const auto offset = item["offset"].as<std::vector<double>>();
    if (foot < 0 || foot > 3 || surface < 0 || offset.size() != 3)
      throw std::runtime_error("invalid_step");
    for (double x : offset)
      if (!std::isfinite(x) || std::abs(x) > .1)
        throw std::runtime_error("template_offset_out_of_initial_m2_envelope");
    out.push_back({static_cast<uint8_t>(foot),
                   static_cast<uint32_t>(surface),
                   {offset[0], offset[1], offset[2]}});
  }
  return out;
}

} // namespace qr_planning
