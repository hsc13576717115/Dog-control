#include "custom_dog_control/precision/PrecisionPlanner.hpp"
#include "custom_dog_control/precision/FootstepExecutor.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include "custom_dog_control/precision/PrecisionWbc.hpp"
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
namespace custom_dog_control {
struct PrecisionPlanner::Impl {
  RobotModelConfig model_config;
  PrecisionConfig config;
  ocs2::CentroidalModelInfo info;
  PrecisionModel preflight;
  std::unique_ptr<PrecisionWbc> check_wbc;
  Impl(const RobotModel &robot, const std::string &urdf,
       const std::string &task, const RobotModelConfig &m,
       const PrecisionConfig &c)
      : model_config(m), config(c), info(robot.info),
        preflight(robot.pin, robot.info, urdf, m, c) {
    std::vector<std::string> names;
    for (auto s : kFootFrameNames)
      names.emplace_back(s);
    ocs2::CentroidalModelPinocchioMapping mapping(info);
    ocs2::PinocchioEndEffectorKinematics ee(robot.pin, mapping, names);
    check_wbc = std::make_unique<PrecisionWbc>(robot.pin, info, ee);
    check_wbc->Configure(config);
    check_wbc->loadTasksSetting(task, false);
  }
  PlanningResult Plan(const PlanningRequest &req, const PrecisionSnapshot &s,
                      const std::vector<PlanningSurface> &regions, double now) {
    PlanningResult result;
    auto reject = [&](PrecisionError error) { result.error = error; };
    if (req.foot >= 4 || !s.ready || !std::isfinite(now) ||
        !SnapshotFresh(s, PrecisionSteadyNow(), config.state_timeout_s)) {
      reject(PrecisionError::STATE_NOT_READY);
      return result;
    }
    const double margin = model_config.foot_radius_m + config.edge_margin_m;
    const auto it =
        std::find_if(regions.begin(), regions.end(),
                     [&](const auto &p) { return p.id == req.surface_id; });
    if (it == regions.end() || !it->known || it->forbidden ||
        !std::isfinite(it->confidence) ||
        it->confidence < config.min_confidence || it->points.size() != 4) {
      reject(PrecisionError::UNKNOWN_SURFACE);
      return result;
    }
    const double age = now - it->observed_at;
    if (age < 0 || age > config.surface_timeout_s) {
      reject(PrecisionError::STALE_SURFACE);
      return result;
    }
    if (!std::isfinite(it->normal.z) || !std::isfinite(it->normal.x) ||
        !std::isfinite(it->normal.y) ||
        std::abs(it->normal.x) + std::abs(it->normal.y) > 1e-3 ||
        std::abs(it->normal.z - 1.) > 1e-3) {
      reject(PrecisionError::M1_HORIZONTAL_SURFACES_ONLY);
      return result;
    }
    for (const auto &point : it->points) {
      if (!std::isfinite(point.z) ||
          std::abs(point.z - it->points.front().z) > 1e-5) {
        reject(PrecisionError::INVALID_PLANE);
        return result;
      }
    }
    Eigen::Vector3d target = req.target;
    std::vector<Eigen::Vector2d> poly;
    for (auto p : it->points)
      poly.push_back({p.x, p.y});
    if (!target.allFinite() ||
        !qr_planning::Inside(poly, target.head<2>(), margin) ||
        std::abs(target.z() - it->points[0].z - model_config.foot_radius_m) >
            config.plane_tolerance_m) {
      reject(PrecisionError::OUTSIDE_ERODED_SURFACE);
      return result;
    }
    if ((target - s.ref.foot[req.foot]).norm() > config.max_step_distance_m ||
        target.z() < model_config.foot_radius_m - .001 ||
        target.z() > model_config.foot_radius_m + config.max_surface_height_m) {
      reject(PrecisionError::M1_MOTION_ENVELOPE);
      return result;
    }
    // M1 accepts only axis-aligned horizontal rectangular fixtures. Check all
    // supplied pads, not a fictitious pad centered at the requested footpoint.
    struct Box {
      Eigen::Vector3d center;
      Eigen::Vector2d size;
      double height;
    };
    std::vector<Box> boxes{{Eigen::Vector3d::Zero(), {10., 10.}, 0.}};
    for (const auto &surface : regions) {
      const double surface_age = now - surface.observed_at;
      if (!surface.known || surface.forbidden ||
          surface.confidence < config.min_confidence ||
          !std::isfinite(surface.confidence) || surface_age < 0 ||
          surface_age > config.surface_timeout_s ||
          surface.points.size() != 4) {
        reject(PrecisionError::M1_INCOMPLETE_FIXTURE_GEOMETRY);
        return result;
      }
      const auto &points = surface.points;
      const Eigen::Vector3d normal(surface.normal.x, surface.normal.y,
                                   surface.normal.z);
      Eigen::Vector2d lo(points[0].x, points[0].y), hi = lo;
      for (const auto &p : points) {
        Eigen::Vector3d xyz(p.x, p.y, p.z);
        if (!xyz.allFinite() || std::abs(p.z - points[0].z) > 1e-5) {
          reject(PrecisionError::M1_INVALID_FIXTURE_GEOMETRY);
          return result;
        }
        lo = lo.cwiseMin(xyz.head<2>());
        hi = hi.cwiseMax(xyz.head<2>());
      }
      const Eigen::Vector2d size = hi - lo;
      if (!normal.allFinite() ||
          (normal - Eigen::Vector3d::UnitZ()).norm() > 1e-3 ||
          size.minCoeff() < 2 * margin || points[0].z < 0 ||
          points[0].z > config.max_surface_height_m) {
        reject(PrecisionError::M1_RECTANGULAR_HORIZONTAL_FIXTURES_ONLY);
        return result;
      }
      const std::array<Eigen::Vector2d, 4> corners{
          lo, {hi.x(), lo.y()}, hi, {lo.x(), hi.y()}};
      for (size_t i = 0; i < 4; ++i) {
        if ((Eigen::Vector2d(points[i].x, points[i].y) - corners[i]).norm() >
            1e-5) {
          reject(PrecisionError::M1_RECTANGULAR_HORIZONTAL_FIXTURES_ONLY);
          return result;
        }
      }
      boxes.push_back({{.5 * (lo.x() + hi.x()), .5 * (lo.y() + hi.y()), 0.},
                       size,
                       points[0].z});
    }
    PrecisionStep step;
    step.shift = config.shift_s;
    step.swing = config.swing_s;
    step.clearance = config.clearance_m;
    step.timeout = config.touchdown_timeout_s;
    step.foot = req.foot;
    step.target = target;
    Eigen::VectorXd initial_q, initial_dq;
    if (!preflight.Inverse(s.ref, s.joints, initial_q, initial_dq)) {
      reject(PrecisionError::INITIAL_STANCE_IK);
      return result;
    }
    const Eigen::Vector3d com = preflight.CenterOfMass(initial_q);
    const Eigen::Vector3d support_center =
        qr_planning::SupportCentroid(s.ref.foot, req.foot);
    std::vector<Eigen::Vector2d> support_polygon;
    for (size_t f = 0; f < 4; ++f)
      if (f != req.foot)
        support_polygon.push_back(s.ref.foot[f].head<2>());
    std::sort(support_polygon.begin(), support_polygon.end(),
              [&](const auto &a, const auto &b) {
                return std::atan2(a.y() - support_center.y(),
                                  a.x() - support_center.x()) <
                       std::atan2(b.y() - support_center.y(),
                                  b.x() - support_center.x());
              });
    bool found_support = false;
    // Shift the actual model COM just far enough inside the support triangle.
    // Moving the base origin to the triangle centroid ignores the COM offset
    // and causes unnecessary leg rotation and rolling of spherical feet.
    for (int i = 0; i <= 100; ++i) {
      const double blend = i / 100.;
      const Eigen::Vector2d next_com =
          (1. - blend) * com.head<2>() + blend * support_center.head<2>();
      if (qr_planning::Inside(support_polygon, next_com,
                              config.support_margin_m)) {
        step.body = s.ref.body;
        step.body.head<2>() += next_com - com.head<2>();
        found_support = true;
        break;
      }
    }
    if (!found_support) {
      reject(PrecisionError::SUPPORT_MARGIN_INFEASIBLE);
      return result;
    }
    auto validate = [&]() {
      FootstepExecutor trial(config);
      trial.Reset(s.ref);
      trial.Start(step, 0.);
      // Sample all reference phases before accepting the action. End-state IK
      // is included; the solver cannot hide an unreachable next stance.
      auto cs = s.contact;
      for (auto &c : cs) {
        c.valid = c.loaded = true;
        c.slipping = false;
      }
      Eigen::VectorXd q, dq;
      const double duration = 2 * step.shift + step.swing + step.timeout +
                              4 * config.preflight_period_s;
      for (double t = 0; t < duration; t += config.preflight_period_s) {
        cs[step.foot].loaded = t >= step.shift + step.swing;
        auto feet = s.ref.foot;
        feet[step.foot] = target;
        const auto ref = trial.Update(t, cs, feet);
        if (trial.phase() == StepPhase::HOLD) {
          reject(PrecisionError::REFERENCE_GENERATION_FAILED);
          return false;
        }
        if (!preflight.Inverse(ref, s.joints, q, dq)) {
          reject(PrecisionError::IK_OR_JOINT_MARGIN);
          return false;
        }
        for (const auto &box : boxes)
          if (!preflight.CollisionFree(q, box.center, box.height, box.size)) {
            reject(PrecisionError::LEG_OR_BODY_COLLISION);
            return false;
          }
        Eigen::VectorXd xd = Eigen::VectorXd::Zero(24),
                        u = Eigen::VectorXd::Zero(24),
                        rbd = Eigen::VectorXd::Zero(36);
        xd.segment<6>(6) = q.head<6>();
        xd.tail<12>() = q.tail<12>();
        rbd.head<3>() = q.segment<3>(3);
        rbd.segment<3>(3) = q.head<3>();
        rbd.segment<12>(6) = q.tail<12>();
        int count = 0;
        for (bool c : ref.contact)
          count += c;
        for (size_t f = 0; f < 4; ++f)
          if (ref.contact[f])
            u(3 * f + 2) = info.robotMass * 9.81 / count;
        check_wbc->Reference(ref);
        check_wbc->update(xd, u, rbd, ContactMode(ref.contact), .001);
        if (!check_wbc->lastSolverSucceeded() ||
            check_wbc->lastEqualityResidual() > config.residual_limit ||
            check_wbc->lastInequalityViolation() > config.residual_limit) {
          reject(PrecisionError::SUPPORT_OR_TORQUE_INFEASIBLE);
          return false;
        }
      }
      if (trial.phase() != StepPhase::DONE) {
        reject(PrecisionError::REFERENCE_GENERATION_FAILED);
        return false;
      }
      return true;
    };
    bool feasible = false;
    for (double lift :
         {0., config.lift_increment_m, 2 * config.lift_increment_m}) {
      step.body.z() = s.ref.body.z() + lift;
      if (validate()) {
        feasible = true;
        break;
      }
    }
    if (!feasible)
      return result;
    result.accepted = true;
    result.error = PrecisionError::NONE;
    result.step = step;
    result.safe_region = it->points;
    const std::array<double, 4> dx{margin, -margin, -margin, margin};
    const std::array<double, 4> dy{margin, margin, -margin, -margin};
    for (size_t k = 0; k < 4; ++k) {
      result.safe_region[k].x += dx[k];
      result.safe_region[k].y += dy[k];
    }
    return result;
  }
};
PrecisionPlanner::PrecisionPlanner(const RobotModel &r, const std::string &u,
                                   const std::string &t,
                                   const RobotModelConfig &m,
                                   const PrecisionConfig &c)
    : impl_(std::make_unique<Impl>(r, u, t, m, c)) {}
PrecisionPlanner::~PrecisionPlanner() = default;
PlanningResult PrecisionPlanner::Plan(const PlanningRequest &r,
                                      const PrecisionSnapshot &s,
                                      const std::vector<PlanningSurface> &p,
                                      double now) {
  return impl_->Plan(r, s, p, now);
}
} // namespace custom_dog_control
