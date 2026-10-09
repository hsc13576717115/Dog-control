// Offline denser replay of a serialized coordinated candidate. No ROS or
// execution authority; success remains a discrete static feasibility result.
#include "CoordinatedScreen.hpp"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace custom_dog_control;
namespace {
Eigen::Vector3d Vec(const YAML::Node &n) {
  if (!n.IsSequence() || n.size() != 3)
    throw std::invalid_argument("expected a three-dimensional vector");
  Eigen::Vector3d v(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
  if (!v.allFinite())
    throw std::invalid_argument("nonfinite vector");
  return v;
}
WholeBodyReference Reference(const YAML::Node &n) {
  WholeBodyReference r;
  r.body = Vec(n["body"]);
  r.euler = Vec(n["euler_zyx"]);
  if (n["feet"].size() != 4 || n["contacts"].size() != 4)
    throw std::invalid_argument("missing foot/contact contract");
  for (size_t f = 0; f < 4; ++f) {
    r.foot[f] = Vec(n["feet"][f]);
    r.contact[f] = n["contacts"][f].as<bool>();
  }
  return r;
}
} // namespace
int main(int argc, char **argv) {
  if (argc != 9) {
    std::cerr << "precision_replay URDF MODEL CONTROL TASK INPUT FIXTURE "
                 "VARIANT OUTPUT\n";
    return 2;
  }
  if (std::filesystem::exists(argv[8]))
    return 2;
  YAML::Node report;
  report["passed"] = false;
  report["dynamic_execution_certified"] = false;
  report["continuous_collision_certified"] = false;
  report["hardware_tested"] = false;
  report["source_report"] = argv[5];
  report["fixture"] = argv[6];
  report["variant"] = argv[7];
  report["subdivisions_per_edge"] = 10;
  try {
    const auto mc = RobotModelConfig::Load(argv[2]);
    const auto config = PrecisionConfig::Load(argv[3]);
    RobotModel robot(argv[1], mc);
    PrecisionModel model(robot.pin, robot.info, argv[1], mc, config);
    boost::property_tree::ptree task;
    boost::property_tree::read_info(argv[4], task);
    const auto input =
        YAML::LoadFile(argv[5])["fixtures"][std::string(argv[6])]["coordinated"]
                               [std::string(argv[7])];
    if (!input || !input["complete_candidate"].as<bool>())
      throw std::invalid_argument("complete candidate required");
    std::vector<ScreenBox> boxes;
    for (const auto &b : input["scene"]) {
      if (b["size"].size() != 2)
        throw std::invalid_argument("invalid box");
      const double height = b["height"].as<double>();
      const double width = b["size"][0].as<double>(),
                   length = b["size"][1].as<double>();
      if (!std::isfinite(height) || height < 0 || !std::isfinite(width) ||
          !std::isfinite(length) || width <= 0 || length <= 0)
        throw std::invalid_argument("nonfinite or invalid scene dimensions");
      boxes.push_back({b["id"].as<std::string>(),
                       Vec(b["center"]),
                       {b["size"][0].as<double>(), b["size"][1].as<double>()},
                       b["height"].as<double>()});
    }
    if (boxes.empty())
      throw std::invalid_argument("missing scene");
    CoordinatedScreen checker(
        robot, model, config,
        task.get<double>("frictionConeTask.frictionCoefficient"), boxes,
        mc.foot_radius_m);
    const auto path = input["furthest_path"];
    if (path.size() < 2 || path.size() > 4096)
      throw std::invalid_argument("invalid path length");
    Eigen::VectorXd q(robot.pin.getModel().nq);
    if (path[0]["q"].size() != static_cast<size_t>(q.size()))
      throw std::invalid_argument("q dimension mismatch");
    for (int k = 0; k < q.size(); ++k)
      q[k] = path[0]["q"][k].as<double>();
    auto previous = Reference(path[0]);
    Eigen::VectorXd next_q;
    if (!checker.Check(previous, checker.Seed(q), next_q))
      throw std::runtime_error("initial state rejected");
    q = next_q;
    size_t checked = 1;
    for (size_t i = 1; i < path.size(); ++i) {
      const auto end = Reference(path[i]);
      for (int sample = 1; sample <= 10; ++sample) {
        const double t = sample / 10.;
        auto ref = end;
        ref.body = (1 - t) * previous.body + t * end.body;
        ref.euler = (1 - t) * previous.euler + t * end.euler;
        for (size_t f = 0; f < 4; ++f) {
          ref.foot[f] = (1 - t) * previous.foot[f] + t * end.foot[f];
          // Lift-off at the final shift knot occurs AFTER the body transfer;
          // do not prematurely remove that support for the entire edge.
          // A moving swing foot is excluded throughout its geometric edge.
          ref.contact[f] =
              sample == 10 ? end.contact[f]
                           : (previous.contact[f] &&
                              (end.contact[f] ||
                               (end.foot[f] - previous.foot[f]).norm() < 1e-9));
        }
        report["edge"] = i;
        report["subsample"] = sample;
        if (!checker.Check(ref, checker.Seed(q), next_q))
          throw std::runtime_error("dense static sample rejected: " +
                                   checker.lastFailure());
        q = next_q;
        ++checked;
      }
      previous = end;
    }
    report["checked_samples"] = checked;
    report["passed"] = true;
  } catch (const std::exception &e) {
    report["error"] = e.what();
  }
  std::ofstream out(argv[8]);
  out << report;
  return out && report["passed"].as<bool>() ? 0 : 1;
}
