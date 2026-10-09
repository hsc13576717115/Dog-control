#pragma once
// Offline bounded beam search. No ROS subscriptions or execution authority.
// The beam and discrete interpolation are deliberately recorded: failure is
// search failure, and a path is not a dynamic or continuous-collision proof.
#include "custom_dog_control/model/StaticSupport.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include "qr_planning/Geometry.hpp"
#include <map>
#include <memory>
#include <pinocchio/multibody/model.hpp>
#include <set>
#include <yaml-cpp/yaml.h>
namespace custom_dog_control {
struct ScreenBox {
  std::string id;
  Eigen::Vector3d center;
  Eigen::Vector2d size;
  double height;
};
struct ScreenStep {
  size_t foot;
  Eigen::Vector3d target;
  double apex;
  double preferred_yaw = std::numeric_limits<double>::quiet_NaN();
  double preferred_height = std::numeric_limits<double>::quiet_NaN();
  bool late_lateral_transfer = false;
};
struct ScreenNode {
  WholeBodyReference ref;
  Eigen::VectorXd q;
  double cost = 0;
  std::shared_ptr<const ScreenNode> parent;
  int step = 0, sample = 0;
};
class CoordinatedScreen {
public:
  CoordinatedScreen(const RobotModel &robot, PrecisionModel &model,
                    const PrecisionConfig &config, double friction,
                    std::vector<ScreenBox> boxes, double foot_radius)
      : robot_(robot), model_(model), config_(config), friction_(friction),
        boxes_(std::move(boxes)), foot_radius_(foot_radius) {}
  bool Check(WholeBodyReference ref, const JointSample &seed,
             Eigen::VectorXd &q) {
    for (size_t f = 0; f < 4; ++f)
      if (ref.contact[f]) {
        bool supported = false;
        for (const auto &box : boxes_) {
          const Eigen::Vector2d inside =
              box.size / 2 -
              Eigen::Vector2d::Constant(foot_radius_ + config_.edge_margin_m);
          supported |= std::abs(ref.foot[f].z() - box.height - foot_radius_) <=
                           config_.plane_tolerance_m &&
                       ((ref.foot[f].head<2>() - box.center.head<2>())
                            .cwiseAbs()
                            .array() <= inside.array() + 1e-9)
                           .all();
        }
        if (!supported)
          return Fail("contact_outside_eroded_support_" + std::to_string(f));
      }
    Eigen::VectorXd dq;
    if (!model_.Inverse(ref, seed, q, dq))
      return Fail("ik_or_joint_margin");
    std::vector<Eigen::Vector2d> support;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (size_t f = 0; f < 4; ++f)
      if (ref.contact[f]) {
        support.push_back(ref.foot[f].head<2>());
        center += support.back();
      }
    if (support.size() < 3)
      return Fail("support_count_below_three");
    center /= support.size();
    std::sort(support.begin(), support.end(), [&](auto a, auto b) {
      return std::atan2(a.y() - center.y(), a.x() - center.x()) <
             std::atan2(b.y() - center.y(), b.x() - center.x());
    });
    if (!qr_planning::Inside(support, model_.CenterOfMass(q).head<2>(),
                             config_.support_margin_m))
      return Fail("support_margin");
    CollisionWitness witness;
    if (!model_.SelfCollisionFree(q, &witness))
      return Fail("self:" + witness.first + ":" + witness.second);
    for (const auto &box : boxes_)
      if (!model_.CollisionFree(q, box.center, box.height, box.size, &witness))
        return Fail("scene:" + witness.first + ":" + box.id);
    const auto forces =
        CheckStaticSupport(robot_, q, ref.contact, friction_, foot_radius_);
    if (!forces.feasible)
      return Fail(forces.infeasible
                      ? "static_force_infeasible"
                      : "force_solver:" +
                            std::to_string(forces.solver_return_code));
    return true;
  }
  const std::string &lastFailure() const { return last_failure_; }
  JointSample Seed(const Eigen::VectorXd &q) const {
    JointSample seed;
    seed.valid.fill(1.);
    for (size_t k = 0; k < 12; ++k)
      seed.position[k] = q(6 + model_.slot(k));
    return seed;
  }
  // Each seed is a candidate initial tripod stance, not a certified approach.
  YAML::Node Run(const std::vector<WholeBodyReference> &initial,
                 const std::vector<ScreenStep> &steps, int beam_width = 48,
                 int samples = 40) {
    failures_.clear();
    std::vector<std::shared_ptr<ScreenNode>> beam;
    for (const auto &ref : initial) {
      for (int attempt = 0; attempt < 3; ++attempt) {
        JointSample seed;
        seed.valid.fill(1.);
        for (size_t f = 0; f < 4; ++f) {
          seed.position[3 * f + 1] =
              attempt == 0 ? .8 : (attempt == 1 ? 1.4 : .4);
          seed.position[3 * f + 2] =
              attempt == 0 ? -1.6 : (attempt == 1 ? -2.4 : -1.0);
        }
        Eigen::VectorXd q;
        if (Check(ref, seed, q)) {
          auto node = std::make_shared<ScreenNode>();
          node->ref = ref;
          node->q = q;
          beam.push_back(node);
          break;
        }
      }
    }
    YAML::Node report;
    report["initial_feasible_states"] = beam.size();
    report["initial_candidates"] = initial.size();
    report["foot_order"] = std::vector<std::string>{"FR", "FL", "RR", "RL"};
    report["coordinate_frame"] = "fixture_world";
    report["foot_radius_m"] = foot_radius_;
    report["edge_margin_m"] = config_.edge_margin_m;
    report["support_margin_m"] = config_.support_margin_m;
    report["friction"] = friction_;
    report["self_geometry"] = "canonical_URDF_visual_CAD_nonadjacent_links";
    report["scene_geometry"] = "all_canonical_URDF_collision_primitives";
    for (const auto &box : boxes_) {
      YAML::Node item;
      item["id"] = box.id;
      item["center"] =
          std::vector<double>{box.center.x(), box.center.y(), box.center.z()};
      item["size"] = std::vector<double>{box.size.x(), box.size.y()};
      item["height"] = box.height;
      report["scene"].push_back(item);
    }
    for (const auto &step : steps) {
      YAML::Node item;
      item["foot"] = step.foot;
      item["target"] = std::vector<double>{step.target.x(), step.target.y(),
                                           step.target.z()};
      item["apex"] = step.apex;
      item["late_lateral_transfer"] = step.late_lateral_transfer;
      if (std::isfinite(step.preferred_height))
        item["preferred_height"] = step.preferred_height;
      if (std::isfinite(step.preferred_yaw))
        item["preferred_yaw"] = step.preferred_yaw;
      report["sequence"].push_back(item);
    }
    for (int k = 0; k < robot_.pin.getModel().nq; ++k) {
      if (k < 6)
        report["q_order"].push_back(std::array<const char *, 6>{
            "x", "y", "z", "yaw", "pitch", "roll"}[k]);
      else {
        for (size_t joint = 1; joint < robot_.pin.getModel().joints.size();
             ++joint)
          if (robot_.pin.getModel().joints[joint].idx_q() == k)
            report["q_order"].push_back(robot_.pin.getModel().names[joint]);
      }
    }
    report["beam_width"] = beam_width;
    report["samples_per_swing"] = samples;
    report["self_collision_checked"] = true;
    report["continuous_collision_certified"] = false;
    report["dynamic_execution_certified"] = false;
    report["mechanical_impossibility_proven"] = false;
    const double initial_yaw = initial.empty() ? 0. : initial.front().euler.x();
    report["initial_heading_rad"] = initial_yaw;
    int reached_step = -1, reached_sample = 0;
    auto last = beam.empty() ? std::shared_ptr<ScreenNode>{} : beam.front();
    for (size_t step_index = 0; step_index < steps.size() && !beam.empty();
         ++step_index) {
      const auto &step = steps[step_index];
      const auto from = beam.front()->ref.foot[step.foot];
      if (step_index > 0) {
        std::vector<std::shared_ptr<ScreenNode>> shifted;
        for (const auto &parent : beam) {
          const Eigen::Vector3d com = model_.CenterOfMass(parent->q);
          const Eigen::Vector3d centroid =
              qr_planning::SupportCentroid(parent->ref.foot, step.foot);
          std::vector<Eigen::Vector2d> polygon;
          for (size_t f = 0; f < 4; ++f)
            if (f != step.foot)
              polygon.push_back(parent->ref.foot[f].head<2>());
          std::sort(
              polygon.begin(), polygon.end(),
              [&](const auto &a, const auto &b) {
                return std::atan2(a.y() - centroid.y(), a.x() - centroid.x()) <
                       std::atan2(b.y() - centroid.y(), b.x() - centroid.x());
              });
          // Minimal transfer matters near a gap: forcing the centroid can
          // overextend the last rear leg even when a safe tripod exists.
          Eigen::Vector3d desired_shift = centroid - com;
          for (int i = 0; i <= 100; ++i) {
            const double blend = i / 100.;
            const Eigen::Vector2d next_com =
                com.head<2>() + blend * (centroid - com).head<2>();
            if (qr_planning::Inside(polygon, next_com,
                                    config_.support_margin_m)) {
              desired_shift.head<2>() = next_com - com.head<2>();
              break;
            }
          }
          for (double x_offset : {-.03, -.015, 0., .015, .03})
            for (double y_offset : {-.03, -.015, 0., .015, .03})
              for (double dz : {-.09, -.06, -.03, 0., .03, .06, .09, .12}) {
                const double dx = desired_shift.x() + x_offset;
                const double dy = desired_shift.y() + y_offset;
                auto ref = parent->ref;
                ref.contact.fill(true);
                ref.contact[step.foot] = false;
                ref.body += Eigen::Vector3d(dx, dy, dz);
                Eigen::VectorXd q;
                if (!Check(ref, Seed(parent->q), q))
                  continue;
                auto previous = parent;
                bool clear = true;
                for (int k = 1; k <= 10; ++k) {
                  auto middle = parent->ref;
                  middle.contact.fill(true);
                  middle.body = parent->ref.body +
                                (k / 10.) * Eigen::Vector3d(dx, dy, dz);
                  Eigen::VectorXd qm;
                  if (!Check(middle, Seed(previous->q), qm)) {
                    clear = false;
                    break;
                  }
                  auto node = std::make_shared<ScreenNode>();
                  node->ref = middle;
                  node->q = qm;
                  node->parent = previous;
                  node->step = step_index;
                  node->sample = -10 + k;
                  node->cost =
                      parent->cost + Eigen::Vector3d(dx, dy, dz).squaredNorm();
                  previous = node;
                }
                if (clear) {
                  previous->ref.contact[step.foot] = false;
                  shifted.push_back(previous);
                }
              }
          // Bound work per shift. Initial candidates retain alternate completed
          // paths; this search is not a completeness or impossibility proof.
          if (shifted.size() > static_cast<size_t>(4 * beam_width))
            break;
        }
        std::sort(
            shifted.begin(), shifted.end(),
            [](const auto &a, const auto &b) { return a->cost < b->cost; });
        if (shifted.size() > static_cast<size_t>(beam_width))
          shifted.resize(beam_width);
        beam = std::move(shifted);
        if (beam.empty()) {
          Fail("no_four_contact_shift_to_next_tripod");
          break;
        }
      }
      std::cerr << "  step " << step_index << " foot " << step.foot
                << " initial beam " << beam.size() << std::endl;
      for (int sample = 1; sample <= samples && !beam.empty(); ++sample) {
        const double t = sample / double(samples);
        Eigen::Vector3d foot = from;
        if (t < .3)
          foot.z() += qr_planning::Quintic(t, .3).p * (step.apex - from.z());
        else if (t < .7) {
          foot =
              from + qr_planning::Quintic(t - .3, .4).p * (step.target - from);
          foot.z() = step.apex;
        } else {
          foot = step.target;
          foot.z() = step.apex + qr_planning::Quintic(t - .7, .3).p *
                                     (step.target.z() - step.apex);
        }
        if (step.late_lateral_transfer && t >= .3) {
          foot = from;
          foot.z() = step.apex;
          if (t < .55)
            foot.x() += qr_planning::Quintic(t - .3, .25).p *
                        (step.target.x() - from.x());
          else if (t < .75) {
            foot.x() = step.target.x();
            foot.y() += qr_planning::Quintic(t - .55, .20).p *
                        (step.target.y() - from.y());
          } else {
            foot = step.target;
            foot.z() = step.apex + qr_planning::Quintic(t - .75, .25).p *
                                       (step.target.z() - step.apex);
          }
        }
        std::vector<std::shared_ptr<ScreenNode>> next;
        std::set<std::array<long, 6>> seen;
        for (const auto &parent : beam)
          for (int move = 0; move < 13; ++move) {
            WholeBodyReference ref = parent->ref;
            ref.contact.fill(true);
            ref.contact[step.foot] = false;
            ref.foot[step.foot] = foot;
            if (move > 0) {
              const int axis = (move - 1) / 2;
              const double sign = move % 2 ? 1. : -1.;
              if (axis < 3)
                ref.body[axis] +=
                    (step.late_lateral_transfer && t > .65 ? .005 : .015) *
                    sign;
              else
                ref.euler[axis - 3] +=
                    (step.late_lateral_transfer && t > .65 ? .02 : .08) * sign;
            }
            if (std::abs(ref.euler[0] - initial_yaw) > .7 ||
                std::abs(ref.euler[1]) > .4 || std::abs(ref.euler[2]) > .24)
              continue;
            std::array<long, 6> key{
                lround(ref.body.x() / .005), lround(ref.body.y() / .005),
                lround(ref.body.z() / .005), lround(ref.euler[1] / .01),
                lround(ref.euler[2] / .01),  lround(ref.euler[0] / .01)};
            if (seen.count(key))
              continue;
            Eigen::VectorXd q;
            if (!Check(ref, Seed(parent->q), q))
              continue;
            if ((q.tail(12) - parent->q.tail(12)).cwiseAbs().maxCoeff() > .3) {
              Fail("joint_edge_jump");
              continue;
            }
            // Validate interpolated transitions as well as graph vertices.
            WholeBodyReference mid = ref;
            mid.body = (ref.body + parent->ref.body) / 2;
            mid.euler = (ref.euler + parent->ref.euler) / 2;
            mid.foot[step.foot] =
                (ref.foot[step.foot] + parent->ref.foot[step.foot]) / 2;
            Eigen::VectorXd qm;
            if (!Check(mid, Seed(parent->q), qm))
              continue;
            if (sample == samples) {
              auto landed = ref;
              landed.contact.fill(true);
              Eigen::VectorXd terminal_q;
              if (!Check(landed, Seed(q), terminal_q))
                continue;
              ref = landed;
            }
            seen.insert(key);
            auto node = std::make_shared<ScreenNode>();
            node->ref = ref;
            node->q = q;
            node->parent = parent;
            node->step = step_index;
            node->sample = sample;
            node->cost = parent->cost +
                         (ref.body - parent->ref.body).squaredNorm() +
                         .002 * (ref.euler - parent->ref.euler).squaredNorm();
            next.push_back(node);
          }
        Eigen::Vector3d destination_center = Eigen::Vector3d::Zero();
        if (!beam.empty()) {
          for (const auto &p : beam.front()->ref.foot)
            destination_center += p / 4.;
          destination_center += (step.target - from) / 4.;
        }
        auto rank = [&](const auto &n) {
          const double yaw_cost =
              std::isfinite(step.preferred_yaw)
                  ? .1 * std::pow(n->ref.euler.x() -
                                      step.preferred_yaw *
                                          std::clamp((t - .3) / .4, 0., 1.),
                                  2)
                  : 0.;
          return n->cost + yaw_cost +
                 (std::isfinite(step.preferred_height)
                      ? 5. *
                            std::pow(n->ref.body.z() - step.preferred_height, 2)
                      : 0.) +
                 5. * (n->ref.body.template head<2>() -
                       destination_center.head<2>())
                          .squaredNorm();
        };
        std::sort(next.begin(), next.end(), [&](const auto &a, const auto &b) {
          return rank(a) < rank(b);
        });
        if (next.size() > static_cast<size_t>(beam_width))
          next.resize(beam_width);
        beam = std::move(next);
        if (!beam.empty()) {
          last = beam.front();
          reached_step = step_index;
          reached_sample = sample;
        }
      }
    }
    report["complete_candidate"] =
        !beam.empty() && reached_step == static_cast<int>(steps.size()) - 1 &&
        reached_sample == samples;
    report["planned_steps"] = steps.size();
    report["landing_regions_checked"] = true;
    report["reached_step"] = reached_step;
    report["reached_sample"] = reached_sample;
    report["failures"] = failures_;
    std::vector<std::shared_ptr<const ScreenNode>> path;
    for (std::shared_ptr<const ScreenNode> n = last; n; n = n->parent)
      path.push_back(n);
    std::reverse(path.begin(), path.end());
    bool attitude_compatible = true;
    double max_pitch = 0., max_roll = 0.;
    for (const auto &n : path) {
      max_pitch = std::max(max_pitch, std::abs(n->ref.euler.y()));
      max_roll = std::max(max_roll, std::abs(n->ref.euler.z()));
      attitude_compatible &= max_pitch <= config_.attitude_limit_rad &&
                             max_roll <= config_.attitude_limit_rad;
      YAML::Node frame;
      frame["step"] = n->step;
      frame["sample"] = n->sample;
      for (bool contact : n->ref.contact)
        frame["contacts"].push_back(contact);
      for (int k = 0; k < 3; ++k) {
        frame["body"].push_back(n->ref.body[k]);
        frame["euler_zyx"].push_back(n->ref.euler[k]);
      }
      for (const auto &f : n->ref.foot) {
        YAML::Node p;
        for (int k = 0; k < 3; ++k)
          p.push_back(f[k]);
        frame["feet"].push_back(p);
      }
      for (int k = 0; k < n->q.size(); ++k)
        frame["q"].push_back(n->q[k]);
      report["furthest_path"].push_back(frame);
    }
    report["max_pitch_rad"] = max_pitch;
    report["max_roll_rad"] = max_roll;
    report["runtime_attitude_compatible"] = attitude_compatible;
    report["runtime_reference_supported"] =
        false; // Current action cannot stream these coordinated knots.
    return report;
  }

private:
  bool Fail(const std::string &reason) {
    last_failure_ = reason;
    ++failures_[reason];
    return false;
  }
  const RobotModel &robot_;
  PrecisionModel &model_;
  PrecisionConfig config_;
  double friction_;
  std::vector<ScreenBox> boxes_;
  double foot_radius_;
  std::map<std::string, int> failures_;
  std::string last_failure_;
};
} // namespace custom_dog_control
