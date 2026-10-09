#pragma once
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include "qr_planning/Geometry.hpp"
namespace custom_dog_control {
class FootstepExecutor {
public:
  explicit FootstepExecutor(PrecisionConfig config = {}) : config_(config) {}
  void Reset(const WholeBodyReference &ref) {
    reference_ = start_ = ref;
    phase_ = StepPhase::STANCE;
    error_ = PrecisionError::NONE;
  }
  void Start(const PrecisionStep &step, double now) {
    tracking_error_since_ = -1;
    step_ = step;
    start_ = reference_;
    entered_ = now;
    phase_ = StepPhase::SHIFT;
    error_ = PrecisionError::NONE;
  }
  void Abort(PrecisionError error) {
    // Never replace current anchors with a flat-ground standing posture.
    error_ = error;
    phase_ = StepPhase::HOLD;
    reference_.probe_foot = -1;
    reference_.probe_force = 0.;
    reference_.body_velocity.setZero();
    reference_.body_acceleration.setZero();
    for (size_t i = 0; i < 4; ++i) {
      reference_.velocity[i].setZero();
      reference_.acceleration[i].setZero();
    }
  }
  const WholeBodyReference &
  Update(double now, const std::array<ContactEstimate, 4> &contacts,
         const std::array<Eigen::Vector3d, 4> &measured) {
    if (phase_ == StepPhase::STANCE || phase_ == StepPhase::DONE ||
        phase_ == StepPhase::HOLD) {
      for (size_t i = 0; i < 4; ++i)
        if (reference_.contact[i] &&
            (!contacts[i].valid || !contacts[i].loaded ||
             contacts[i].slipping)) {
          Abort(PrecisionError::SUPPORT_UNCONFIRMED);
          break;
        }
      return reference_;
    }
    for (size_t i = 0; i < 4; ++i)
      if (i != step_.foot &&
          (!contacts[i].valid || !contacts[i].loaded || contacts[i].slipping)) {
        Abort(PrecisionError::SUPPORT_UNCONFIRMED);
        return reference_;
      }
    const double t = now - entered_;
    const auto f = step_.foot;
    if (phase_ == StepPhase::SHIFT) {
      auto b = qr_planning::Quintic(t, step_.shift);
      const Eigen::Vector3d d = step_.body - start_.body;
      reference_.body = start_.body + b.p * d;
      reference_.body_velocity = b.v * d;
      reference_.body_acceleration = b.a * d;
      if (t >= step_.shift) {
        phase_ = StepPhase::SWING;
        entered_ = now;
        reference_.contact[f] = false;
      }
    } else if (phase_ == StepPhase::SWING) {
      auto b = qr_planning::Quintic(t, step_.swing);
      const Eigen::Vector3d d = step_.target - start_.foot[f];
      reference_.foot[f] = start_.foot[f] + b.p * d;
      reference_.velocity[f] = b.v * d;
      reference_.acceleration[f] = b.a * d;
      // Smooth lift with zero p/v/a at both endpoints.
      const double s = std::clamp(t / step_.swing, 0., 1.);
      const double h = step_.clearance;
      reference_.foot[f].z() += 64 * h * s * s * s * std::pow(1 - s, 3);
      reference_.velocity[f].z() +=
          192 * h * s * s * (1 - s) * (1 - s) * (1 - 2 * s) / step_.swing;
      reference_.acceleration[f].z() += 384 * h * s * (1 - s) *
                                        (1 - 5 * s + 5 * s * s) /
                                        (step_.swing * step_.swing);
      if (t > config_.early_contact_delay_s &&
          t < step_.swing * config_.early_contact_fraction &&
          contacts[f].loaded) {
        Abort(PrecisionError::EARLY_CONTACT);
        return reference_;
      }
      // IMU + joint FK can detect a sustained blocked swing without pretending
      // that its cause or a ground-support contact has been measured.
      if ((measured[f] - reference_.foot[f]).norm() > config_.swing_error_m) {
        if (tracking_error_since_ < 0)
          tracking_error_since_ = now;
        if (now - tracking_error_since_ >= config_.swing_error_dwell_s) {
          Abort(PrecisionError::SWING_TRACKING_ERROR);
          return reference_;
        }
      } else
        tracking_error_since_ = -1;
      if (t >= step_.swing) {
        phase_ = StepPhase::CONFIRM;
        entered_ = now;
        confirmed_since_ = -1;
        candidate_since_ = -1;
      }
    } else if (phase_ == StepPhase::CONFIRM) {
      const bool on_target =
          (measured[f] - step_.target).norm() < config_.target_tolerance_m;
      // A bounded preload produces observable joint-effort evidence. It is not
      // a contact declaration; only persistent residual + geometry admits load
      // transfer.
      if (!reference_.contact[f]) {
        reference_.probe_foot = static_cast<int>(f);
        reference_.probe_force = config_.probe_force_n *
                                 std::clamp(t / config_.probe_ramp_s, 0., 1.);
        if (measured[f].z() < step_.target.z() - config_.probe_travel_m) {
          Abort(PrecisionError::PROBE_TRAVEL_LIMIT);
          return reference_;
        }
        if (on_target && contacts[f].valid &&
            contacts[f].estimated_force.z() > config_.candidate_force_n) {
          if (candidate_since_ < 0)
            candidate_since_ = now;
          if (now - candidate_since_ > config_.candidate_dwell_s) {
            reference_.contact[f] = true;
            reference_.probe_foot = -1;
            reference_.probe_force = 0.;
          }
        } else
          candidate_since_ = -1;
      }
      if (contacts[f].loaded && !contacts[f].slipping && on_target) {
        if (confirmed_since_ < 0)
          confirmed_since_ = now;
        reference_.contact[f] = true;
        if (now - confirmed_since_ > config_.confirm_dwell_s) {
          phase_ = StepPhase::RESTORE;
          entered_ = now;
        }
      } else
        confirmed_since_ = -1;
      if (t > step_.timeout)
        Abort(PrecisionError::TOUCHDOWN_TIMEOUT);
    } else if (phase_ == StepPhase::RESTORE) {
      if (!contacts[f].loaded) {
        Abort(PrecisionError::TOUCHDOWN_LOST);
        return reference_;
      }
      auto b = qr_planning::Quintic(t, step_.shift);
      const Eigen::Vector3d d = start_.body - step_.body;
      reference_.body = step_.body + b.p * d;
      reference_.body_velocity = b.v * d;
      reference_.body_acceleration = b.a * d;
      if (t >= step_.shift)
        phase_ = StepPhase::DONE;
    }
    return reference_;
  }
  StepPhase phase() const { return phase_; }
  const char *error() const { return ErrorName(error_); }
  PrecisionError error_code() const { return error_; }
  uint64_t id() const { return step_.id; }
  const WholeBodyReference &reference() const { return reference_; }

private:
  PrecisionConfig config_;
  WholeBodyReference reference_, start_;
  PrecisionStep step_;
  StepPhase phase_ = StepPhase::STANCE;
  PrecisionError error_ = PrecisionError::NONE;
  double entered_ = 0, confirmed_since_ = -1, candidate_since_ = -1;
  double tracking_error_since_ = -1;
};
} // namespace custom_dog_control
