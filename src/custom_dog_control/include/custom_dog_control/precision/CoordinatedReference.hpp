#pragma once
#include "custom_dog_control/precision/PrecisionConfig.hpp"
#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include "qr_planning/Geometry.hpp"
namespace custom_dog_control {
// Check before exposing a command to the control thread. Each segment is a
// stop-to-stop quintic: C2, bounded work, no extrapolation or Euler wrapping.
inline bool ValidTrajectory(const PrecisionStep &s,
                            const WholeBodyReference &initial,
                            const PrecisionConfig &c) {
  const auto &p = s.trajectory;
  if (!std::isfinite(s.shift) || !std::isfinite(s.swing) ||
      !std::isfinite(s.timeout) || s.shift <= 0 || s.swing <= 0 ||
      s.timeout <= 0 || !s.target.allFinite() || !s.body_finish.allFinite() ||
      s.foot >= 4 || p.size < 3 || p.size > p.capacity || p.lift_index == 0 ||
      p.lift_index >= p.size - 1)
    return false;
  const auto &first = p.knots[0];
  const auto &last = p.knots[p.size - 1];
  if (first.time != 0 || (first.body - initial.body).norm() > c.body_drift_m ||
      (first.euler - initial.euler).norm() > c.body_drift_m ||
      (first.foot - initial.foot[s.foot]).norm() > c.target_tolerance_m ||
      (last.foot - s.target).norm() > 1e-6 ||
      last.time > c.trajectory_duration_s ||
      std::abs(s.shift - p.knots[p.lift_index].time) > 1e-8 ||
      std::abs(s.swing - (last.time - s.shift)) > 1e-8)
    return false;
  for (size_t i = 0; i < p.size; ++i) {
    const auto &k = p.knots[i];
    if (!std::isfinite(k.time) || !k.body.allFinite() || !k.euler.allFinite() ||
        !k.foot.allFinite() || std::abs(k.euler.y()) > c.attitude_limit_rad ||
        std::abs(k.euler.z()) > c.attitude_limit_rad ||
        (i <= p.lift_index && (k.foot - first.foot).norm() > 1e-6))
      return false;
    if (!i)
      continue;
    const auto &a = p.knots[i - 1];
    const double dt = k.time - a.time;
    if (dt < 2 * c.preflight_period_s)
      return false;
    // Exact extrema of unit quintic speed (1.875) / acceleration (5.774).
    for (const auto &pair :
         {std::make_pair((k.body - a.body).norm(), c.trajectory_body_speed_mps),
          std::make_pair((k.foot - a.foot).norm(), c.trajectory_foot_speed_mps),
          std::make_pair((k.euler - a.euler).norm(),
                         c.trajectory_euler_speed_rps)})
      if (1.875 * pair.first / dt > pair.second ||
          5.774 * pair.first / (dt * dt) > 4 * pair.second)
        return false;
  }
  return true;
}
inline void SampleTrajectory(const CoordinatedReference &p, double t,
                             bool swing, size_t f, WholeBodyReference &ref) {
  const size_t begin = swing ? p.lift_index : 0;
  const size_t end = swing ? p.size - 1 : p.lift_index;
  t = std::clamp(t + p.knots[begin].time, p.knots[begin].time,
                 p.knots[end].time);
  size_t i = begin;
  while (i + 1 < end && t > p.knots[i + 1].time)
    ++i;
  const auto &a = p.knots[i];
  const auto &b = p.knots[i + 1];
  const auto q = qr_planning::Quintic(t - a.time, b.time - a.time);
  ref.body = a.body + q.p * (b.body - a.body);
  ref.body_velocity = q.v * (b.body - a.body);
  ref.body_acceleration = q.a * (b.body - a.body);
  ref.euler = a.euler + q.p * (b.euler - a.euler);
  ref.euler_velocity = q.v * (b.euler - a.euler);
  ref.euler_acceleration = q.a * (b.euler - a.euler);
  if (swing) {
    ref.foot[f] = a.foot + q.p * (b.foot - a.foot);
    ref.velocity[f] = q.v * (b.foot - a.foot);
    ref.acceleration[f] = q.a * (b.foot - a.foot);
  }
}
} // namespace custom_dog_control
