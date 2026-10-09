#pragma once
#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include <legged_wbc/WeightedWbc.h>
namespace custom_dog_control {
class PrecisionWbc final : public legged::WeightedWbc {
public:
  using legged::WeightedWbc::WeightedWbc;
  void Reference(const WholeBodyReference &ref) { ref_ = ref; }

protected:
  legged::Task formulateConstraints() override;
  legged::Task formulateWeightedTasks(const ocs2::vector_t &,
                                      const ocs2::vector_t &,
                                      ocs2::scalar_t) override;

private:
  WholeBodyReference ref_;
};
} // namespace custom_dog_control
