#pragma once
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include <legged_wbc/WeightedWbc.h>
namespace custom_dog_control {
class PrecisionWbc final : public legged::WeightedWbc {
public:
  using legged::WeightedWbc::WeightedWbc;
  void Configure(const PrecisionConfig &config, double sole_radius) {
    config_ = config;
    sole_radius_ = sole_radius;
  }
  void Reference(const WholeBodyReference &ref) { ref_ = ref; }

protected:
  legged::Task formulateConstraints() override;
  legged::Task formulateWeightedTasks(const ocs2::vector_t &,
                                      const ocs2::vector_t &,
                                      ocs2::scalar_t) override;

private:
  WholeBodyReference ref_;
  PrecisionConfig config_;
  double sole_radius_ = 0.;
};
} // namespace custom_dog_control
