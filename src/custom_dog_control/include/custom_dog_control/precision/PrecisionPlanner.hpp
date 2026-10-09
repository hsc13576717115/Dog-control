#pragma once
#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionState.hpp"
#include <memory>
#include <vector>
namespace custom_dog_control {
struct SurfacePoint {
  double x = 0, y = 0, z = 0;
};
struct PlanningSurface {
  uint32_t id = 0;
  bool known = false, forbidden = true;
  double confidence = 0, observed_at = 0;
  SurfacePoint normal;
  std::vector<SurfacePoint> points;
};
struct PlanningRequest {
  size_t foot = 0;
  uint32_t surface_id = 0;
  Eigen::Vector3d target;
};
struct PlanningResult {
  bool accepted = false;
  PrecisionError error = PrecisionError::NONE;
  PrecisionStep step;
  std::vector<SurfacePoint> safe_region;
};
// Non-RT, serial calls; owns independent preflight model and QP data.
class PrecisionPlanner {
public:
  PrecisionPlanner(const RobotModel &, const std::string &urdf,
                   const std::string &task, const RobotModelConfig &,
                   const PrecisionConfig &);
  ~PrecisionPlanner();
  PlanningResult Plan(const PlanningRequest &, const PrecisionSnapshot &,
                      const std::vector<PlanningSurface> &, double now);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace custom_dog_control
