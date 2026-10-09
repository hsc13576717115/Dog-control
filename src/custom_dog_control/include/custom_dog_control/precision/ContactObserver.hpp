#pragma once
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include <algorithm>
namespace custom_dog_control {
class ContactObserver {
public:
  explicit ContactObserver(PrecisionConfig config = {}) : config_(config) {}
  // Caller supplies model residual forces; without valid effort there is no
  // load confirmation. Kinematic stillness alone must not prove support.
  ContactEstimate Update(double stamp, const Eigen::Vector3d &force,
                         const Eigen::Vector3d &velocity, bool valid) {
    ContactEstimate out;
    out.stamp = stamp;
    if (!valid || !force.allFinite() || !velocity.allFinite() ||
        stamp <= last_stamp_) {
      if (!valid) {
        loaded_ = false;
        candidate_since_ = -1;
      }
      return out;
    }
    const double dt =
        last_stamp_ > 0 ? std::min(stamp - last_stamp_, .02) : .004;
    last_stamp_ = stamp;
    filtered_ += (force - filtered_) * dt / (config_.force_filter_s + dt);
    const bool candidate =
        filtered_.z() > (loaded_ ? config_.load_off_n : config_.load_on_n) &&
        velocity.norm() < config_.contact_speed_mps;
    if (candidate != candidate_) {
      candidate_ = candidate;
      candidate_since_ = stamp;
    }
    if (candidate_since_ >= 0 &&
        stamp - candidate_since_ >=
            (candidate ? config_.load_dwell_s : config_.unload_dwell_s))
      loaded_ = candidate;
    out.valid = true;
    out.loaded = loaded_;
    out.estimated_force = filtered_;
    const bool slip_candidate =
        loaded_ && velocity.head<2>().norm() > config_.slip_speed_mps;
    if (!slip_candidate)
      slip_since_ = -1;
    else if (slip_since_ < 0)
      slip_since_ = stamp;
    out.slipping =
        slip_since_ >= 0 && stamp - slip_since_ > config_.slip_dwell_s;
    out.probability = std::clamp((filtered_.z() - config_.load_off_n) /
                                     (4 * config_.load_off_n),
                                 0., 1.);
    if (out.slipping)
      out.probability *= .2;
    return out;
  }

private:
  PrecisionConfig config_;
  double last_stamp_ = -1, candidate_since_ = -1, slip_since_ = -1;
  bool candidate_ = false, loaded_ = false;
  Eigen::Vector3d filtered_ = Eigen::Vector3d::Zero();
};
} // namespace custom_dog_control
