// Offline local-fixture screening, not an executable obstacle controller.
#include "CoordinatedScreen.hpp"
#include "custom_dog_control/model/RobotModel.hpp"
#include "custom_dog_control/model/StaticSupport.hpp"
#include "custom_dog_control/precision/PrecisionModel.hpp"
#include "custom_dog_control/precision/PrecisionState.hpp"
#include "qr_planning/Geometry.hpp"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <yaml-cpp/yaml.h>
using namespace custom_dog_control;
struct Box {
  Eigen::Vector3d center;
  Eigen::Vector2d size;
  double height;
};
struct Fixture {
  std::string name;
  std::array<Eigen::Vector3d, 4> feet;
  Eigen::Vector3d target;
  double apex;
  std::vector<Box> boxes;
  std::vector<double> x, y, z;
};
int main(int argc, char **argv) {
  if (argc < 7 || argc > 10) {
    std::cerr << "precision_envelope URDF MODEL_CONFIG CONTROL_CONFIG "
                 "TASK_INFO RULES.yaml OUTPUT.yaml [MODE [FIXTURE "
                 "[STANCE_HALF_WIDTH]]]\n";
    return 2;
  }
  if (std::filesystem::exists(argv[6]) ||
      std::filesystem::exists(std::string(argv[6]) + ".partial")) {
    std::cerr << "Output exists\n";
    return 2;
  }
  try {
    const std::string mode = argc >= 8 ? argv[7] : "fixed";
    if (mode != "fixed" && mode != "--coordinated" && mode != "--followup" &&
        mode != "--followup-yaw" && mode != "--followup-directed" &&
        mode != "--followup-prepared" && mode != "--followup-sideways")
      throw std::invalid_argument("unknown search mode");
    const bool followup = mode.rfind("--followup", 0) == 0;
    const bool prepared = mode == "--followup-prepared";
    const bool sideways = mode == "--followup-sideways";
    const bool directed = mode == "--followup-directed" || prepared;
    const bool turn_body = mode == "--followup-yaw" || directed;
    const std::string selected_fixture = argc >= 9 ? argv[8] : "";
    if (!selected_fixture.empty() &&
        selected_fixture != "platform_to_one_stone" &&
        selected_fixture != "wall_front_leg" &&
        selected_fixture != "gap_front_leg")
      throw std::invalid_argument("unknown fixture");
    const std::string selected_variant = argc == 10 ? argv[9] : "";
    bool valid_variant = selected_variant.empty();
    for (double length : {.24, .30, .37})
      for (double width : {.12, .16, .20, .24, .28})
        valid_variant |= selected_variant ==
                         std::to_string(length) + "_" + std::to_string(width);
    if (!valid_variant)
      throw std::invalid_argument("unknown stance variant");
    auto m = RobotModelConfig::Load(argv[2]);
    auto c = PrecisionConfig::Load(argv[3]);
    RobotModel robot(argv[1], m);
    PrecisionModel model(robot.pin, robot.info, argv[1], m, c);
    boost::property_tree::ptree task;
    boost::property_tree::read_info(argv[4], task);
    const double friction =
        task.get<double>("frictionConeTask.frictionCoefficient");
    const auto rules = YAML::LoadFile(argv[5]);
    const auto stone = rules["obstacles"]["stones"];
    const auto wall = rules["obstacles"]["wall"];
    const auto gap = rules["obstacles"]["gap"];
    const double r = m.foot_radius_m, sh = stone["block_size"][2].as<double>(),
                 wh = wall["height"].as<double>(),
                 gh = gap["platform_height"].as<double>();
    const double gw = gap["width"].as<double>(),
                 pw = gap["platform_size"][0].as<double>(),
                 shore = gw / 2 + r + c.edge_margin_m;
    const double rear = shore + .37;
    // Dimensions match selected nominal rule parameters, while placements and
    // tested body poses are explicitly engineering fixtures, not the full
    // course.
    const std::vector<Fixture> fixtures{
        {"platform_to_one_stone",
         {{{-.05, -.2, sh + r},
           {-.05, .2, sh + r},
           {-.4, -.2, sh + r},
           {-.4, .2, sh + r}}},
         {.3, -.2, sh + r},
         sh + .08 + r,
         {{{-.4, 0, 0},
           {stone["platform_size"][0].as<double>(),
            stone["platform_size"][1].as<double>()},
           sh},
          {{.3, -.2, 0},
           {stone["block_size"][0].as<double>(),
            stone["block_size"][1].as<double>()},
           sh}},
         {-.30, -.25, -.2, -.15, -.1, -.05},
         {0, .05, .1, .15, .2},
         {.36, .40, .44, .48, .52}},
        {"wall_front_leg",
         {{{-.10, -.18, r}, {-.10, .18, r}, {-.46, -.18, r}, {-.46, .18, r}}},
         {.10, -.18, r},
         wh + .02 + r,
         {{{0, 0, 0},
           {wall["thickness"].as<double>(), wall["width"].as<double>()},
           wh}},
         {-.35, -.30, -.25, -.20, -.15},
         {0, .05, .1},
         {.30, .34, .38, .42}},
        {"gap_front_leg",
         {{{-shore, -.18, gh + r},
           {-shore, .18, gh + r},
           {-rear, -.18, gh + r},
           {-rear, .18, gh + r}}},
         {shore, -.18, gh + r},
         gh + .07 + r,
         {{{-(gw + pw) / 2, 0, 0},
           {pw, gap["platform_size"][1].as<double>()},
           gh},
          {{(gw + pw) / 2, 0, 0},
           {pw, gap["platform_size"][1].as<double>()},
           gh}},
         {-.55, -.50, -.45, -.40, -.35, -.30, -.25, -.20},
         {0, .05, .1, .15},
         {.54, .58, .62, .66}}};
    YAML::Node report;
    report["scope"] =
        "Offline fixed-body baseline and optional finite coordinated "
        "templates; no dynamic execution certification";
    report["search_mode"] = mode;
    report["hardware_tested"] = false;
    report["fixed_body_self_collision_checked"] = false;
    report["reference_samples"] = 41;
    report["trajectory_templates"] =
        std::vector<std::string>{"arch", "lift_translate_lower"};
    report["joint_initializations"] = 3;
    report["support_margin_m"] = c.support_margin_m;
    report["static_force_friction"] = friction;
    report["static_force_constraint_residual_limit"] = 1e-5;
    report["rules_version"] = rules["version"];
    for (const auto &f : fixtures) {
      if (!selected_fixture.empty() && f.name != selected_fixture)
        continue;
      size_t candidates = 0, feasible = 0;
      std::map<std::string, int> failures;
      std::map<int, int> force_solver_codes, failure_phase_samples;
      YAML::Node selected;
      for (double x : f.x)
        for (double y : f.y)
          for (double z : f.z) {
            ++candidates;
            bool accepted = false;
            for (int attempt = 0; attempt < 6 && !accepted; ++attempt) {
              const int seed_id = attempt % 3, path_id = attempt / 3;
              JointSample seed;
              seed.valid.fill(1.);
              for (size_t leg = 0; leg < 4; ++leg) {
                seed.position[leg * 3 + 1] =
                    seed_id == 0 ? .8 : (seed_id == 1 ? 1.4 : .4);
                seed.position[leg * 3 + 2] =
                    seed_id == 0 ? -1.6 : (seed_id == 1 ? -2.4 : -1.0);
              }
              WholeBodyReference ref;
              ref.body = {x, y, z};
              ref.foot = f.feet;
              ref.contact[0] = false;
              std::vector<Eigen::Vector2d> polygon;
              for (size_t leg = 1; leg < 4; ++leg)
                polygon.push_back(f.feet[leg].head<2>());
              const auto centroid = qr_planning::SupportCentroid(f.feet, 0);
              std::sort(polygon.begin(), polygon.end(), [&](auto a, auto b) {
                return std::atan2(a.y() - centroid.y(), a.x() - centroid.x()) <
                       std::atan2(b.y() - centroid.y(), b.x() - centroid.x());
              });
              std::string error;
              int evaluated_sample = 0;
              for (int sample = 0; sample <= 40; ++sample) {
                evaluated_sample = sample;
                const double t = sample / 40.;
                ref.foot[0] = (1 - t) * f.feet[0] + t * f.target;
                ref.foot[0].z() +=
                    4 * t * (1 - t) *
                    (f.apex - std::max(f.feet[0].z(), f.target.z()));
                if (path_id == 1) {
                  ref.foot[0] = f.feet[0];
                  if (t < .3)
                    ref.foot[0].z() += qr_planning::Quintic(t, .3).p *
                                       (f.apex - f.feet[0].z());
                  else if (t < .7) {
                    ref.foot[0] =
                        f.feet[0] + qr_planning::Quintic(t - .3, .4).p *
                                        (f.target - f.feet[0]);
                    ref.foot[0].z() = f.apex;
                  } else {
                    ref.foot[0] = f.target;
                    ref.foot[0].z() =
                        f.apex + qr_planning::Quintic(t - .7, .3).p *
                                     (f.target.z() - f.apex);
                  }
                }
                Eigen::VectorXd q, dq;
                if (!model.Inverse(ref, seed, q, dq)) {
                  error = sample == 0 ? "initial_pose_ik_or_joint_margin"
                                      : "swing_path_ik_or_joint_margin";
                  break;
                }
                if (!qr_planning::Inside(polygon,
                                         model.CenterOfMass(q).head<2>(),
                                         c.support_margin_m)) {
                  error = "static_support_margin";
                  break;
                }
                bool clear = model.CollisionFree(q, {0, 0, 0}, 0, {10, 10});
                for (const auto &box : f.boxes)
                  clear = clear && model.CollisionFree(q, box.center,
                                                       box.height, box.size);
                if (!clear) {
                  error = "body_leg_or_swing_collision";
                  break;
                }
                const auto support =
                    CheckStaticSupport(robot, q, ref.contact, friction);
                if (!support.feasible) {
                  ++force_solver_codes[support.solver_return_code];
                  error = support.infeasible
                              ? "static_force_infeasible"
                              : "static_force_solver_or_residual_failure";
                  break;
                }
                for (size_t k = 0; k < 12; ++k)
                  seed.position[k] = q(6 + model.slot(k));
              }
              if (error.empty()) {
                accepted = true;
                ++feasible;
                if (selected.IsNull()) {
                  selected["body"].push_back(x);
                  selected["body"].push_back(y);
                  selected["body"].push_back(z);
                  selected["seed"] = seed_id;
                  selected["trajectory"] =
                      path_id == 0 ? "arch" : "lift_translate_lower";
                }
              } else {
                ++failures[error];
                ++failure_phase_samples[evaluated_sample];
              }
            }
          }
      auto item = report["fixtures"][f.name];
      item["candidate_body_poses"] = candidates;
      item["screened_feasible_poses"] = feasible;
      item["failed_seed_attempts"] = failures;
      item["failed_force_solver_codes"] = force_solver_codes;
      item["first_failure_phase_sample_histogram"] = failure_phase_samples;
      item["first_candidate"] = selected;
      for (const auto &foot : f.feet) {
        YAML::Node position;
        for (int k = 0; k < 3; ++k)
          position.push_back(foot[k]);
        item["initial_feet"].push_back(position);
      }
      for (int k = 0; k < 3; ++k)
        item["target"].push_back(f.target[k]);
      item["apex_z"] = f.apex;
      item["body_x_samples"] = f.x;
      item["body_y_samples"] = f.y;
      item["body_z_samples"] = f.z;
      if (mode != "fixed") {
        std::vector<ScreenBox> scene{{"floor", {0, 0, 0}, {10, 10}, 0}};
        for (size_t i = 0; i < f.boxes.size(); ++i)
          scene.push_back({"fixture_" + std::to_string(i), f.boxes[i].center,
                           f.boxes[i].size, f.boxes[i].height});
        for (double stance_length : {.24, .30, .37})
          for (double half_width :
               (sideways ? std::vector<double>{.12, .16, .20}
                         : std::vector<double>{.20, .24, .28})) {
            const std::string variant = std::to_string(stance_length) + "_" +
                                        std::to_string(half_width);
            if (!selected_variant.empty() && selected_variant != variant)
              continue;
            auto geometry = scene;
            if (followup && f.name == "platform_to_one_stone")
              geometry.push_back({"second_stone", {.3, .2, 0}, {.2, .2}, sh});
            CoordinatedScreen screen(robot, model, c, friction, geometry,
                                     m.foot_radius_m);
            std::vector<WholeBodyReference> initial;
            auto feet = f.feet;
            for (size_t leg = 0; leg < 4; ++leg) {
              feet[leg].y() = (leg % 2 ? 1. : -1.) * half_width;
              if (leg >= 2)
                feet[leg].x() = feet[leg - 2].x() - stance_length;
            }
            if (sideways) {
              if (f.name != "wall_front_leg")
                throw std::invalid_argument("sideways fixture is wall only");
              feet = {{{-.1, stance_length / 2, r},
                       {-.1 - 2 * half_width, stance_length / 2, r},
                       {-.1, -stance_length / 2, r},
                       {-.1 - 2 * half_width, -stance_length / 2, r}}};
            }
            for (double x : f.x)
              for (double y :
                   (sideways ? std::vector<double>{-.1, -.05, 0., .05, .1}
                             : f.y))
                for (double z : f.z)
                  for (double pitch : {-.32, -.16, 0., .16, .32})
                    for (double yaw :
                         (sideways
                              ? std::vector<double>{1.5707963267948966}
                              : (turn_body ? std::vector<double>{0., .35, .6}
                                           : std::vector<double>{0.}))) {
                      WholeBodyReference ref;
                      ref.body = {x, y, z};
                      ref.euler[1] = pitch;
                      ref.euler[0] = yaw;
                      ref.foot = feet;
                      ref.contact[prepared && f.name != "platform_to_one_stone"
                                      ? 2
                                      : 0] = false;
                      initial.push_back(ref);
                    }
            std::cerr << "Coordinated search: " << f.name << " " << variant
                      << std::endl;
            Eigen::Vector3d target = f.target;
            if (f.name != "platform_to_one_stone")
              target.y() = -half_width;
            if (directed && f.name == "wall_front_leg")
              target.x() = .23;
            std::vector<ScreenStep> sequence{{0, target, f.apex}};
            if (followup) {
              if (f.name == "platform_to_one_stone") {
                sequence.push_back({2, {-.05, -half_width, sh + r}, f.apex});
                sequence.push_back({1, {.35, .2, sh + r}, f.apex});
                sequence.push_back({3, {-.05, half_width, sh + r}, f.apex});
                sequence.push_back({0, {.35, -.2, sh + r}, f.apex});
                sequence.push_back({2, {.25, -.2, sh + r}, f.apex});
                sequence.push_back({3, {.25, .2, sh + r}, f.apex});
              } else if (f.name == "gap_front_leg") {
                sequence.push_back(
                    {2, {-shore - .005, -half_width, gh + r}, f.apex});
                sequence.push_back({1, {shore, half_width, gh + r}, f.apex});
                sequence.push_back(
                    {3, {-shore - .005, half_width, gh + r}, f.apex});
                sequence.push_back(
                    {0, {shore + .25, -half_width, gh + r}, f.apex});
                sequence.push_back(
                    {1, {shore + .25, half_width, gh + r}, f.apex});
                sequence.push_back({2, {shore, -half_width, gh + r}, f.apex});
                sequence.push_back({3, {shore, half_width, gh + r}, f.apex});
              } else {
                sequence.push_back({1, {.10, half_width, r}, f.apex});
                sequence.push_back({0, {.3, -half_width, r}, r + .08});
                sequence.push_back({1, {.3, half_width, r}, r + .08});
                sequence.push_back({2, {.10, -half_width, r}, f.apex});
                sequence.push_back({3, {.10, half_width, r}, f.apex});
              }
            }
            if (prepared && f.name == "gap_front_leg") {
              sequence = {{2, {-shore - .08, -half_width, gh + r}, f.apex},
                          {0, {shore, -.14, gh + r}, f.apex},
                          {3, {-shore - .08, half_width, gh + r}, f.apex},
                          {1, {shore, .14, gh + r}, f.apex},
                          {2, {-shore - .005, -half_width, gh + r}, f.apex},
                          {3, {-shore - .005, half_width, gh + r}, f.apex},
                          {0, {shore + .25, -.14, gh + r}, f.apex},
                          {2, {shore, -.18, gh + r}, f.apex},
                          {3, {shore, .18, gh + r}, f.apex},
                          {1, {shore + .25, .14, gh + r}, f.apex}};
            }
            if (prepared && f.name == "wall_front_leg") {
              sequence = {{2, {-.18, -half_width, r}, r + .06},
                          {0, {.247, .172, r}, f.apex, .62},
                          {3, {-.18, half_width, r}, r + .06},
                          {1, {.247, .35, r}, f.apex},
                          {2, {-.09, -half_width, r}, r + .06},
                          {3, {-.09, half_width, r}, r + .06},
                          {0, {.45, -.1, r}, r + .08},
                          {2, {.23, -.16, r}, f.apex},
                          {3, {.23, .16, r}, f.apex},
                          {1, {.45, .1, r}, r + .08}};
            }
            if (sideways) {
              const double y = stance_length / 2.;
              sequence = {{0, {.10, y, r}, f.apex},  {2, {.10, -y, r}, f.apex},
                          {0, {.28, y, r}, r + .08}, {2, {.28, -y, r}, r + .08},
                          {1, {.10, y, r}, f.apex},  {3, {.10, -y, r}, f.apex}};
            }
            if (f.name == "wall_front_leg")
              for (auto &step : sequence) {
                step.preferred_height = .36;
                step.late_lateral_transfer = prepared && step.apex > .3;
              }
            item["coordinated"][variant] =
                screen.Run(initial, sequence,
                           prepared && f.name == "wall_front_leg" ? 96 : 48,
                           prepared && f.name == "wall_front_leg" ? 160 : 40);
            auto r = item["coordinated"][variant];
            r["stance_length_m"] = stance_length;
            r["half_width_m"] = half_width;
            {
              const std::string partial_path =
                  std::string(argv[6]) + ".partial";
              const std::string temporary_path = partial_path + ".tmp";
              {
                std::ofstream partial(temporary_path);
                partial << report;
                if (!partial)
                  throw std::runtime_error("cannot write partial report");
              }
              std::filesystem::rename(temporary_path, partial_path);
            }
            std::cerr << "Result: " << r["complete_candidate"] << " phase "
                      << r["reached_sample"] << std::endl;
          }
      }
      item["mechanical_impossibility_proven"] = false;
      item["interpretation"] =
          feasible ? "candidate requires approach, self-collision, dynamic "
                     "tracking and subsequent/rear-step validation"
                   : "bounded fixed-body template failed; not proof of "
                     "mechanical impossibility";
    }
    std::ofstream output(argv[6]);
    output << report;
    if (!output)
      return 2;
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
