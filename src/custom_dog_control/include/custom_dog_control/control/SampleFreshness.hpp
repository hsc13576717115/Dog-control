#pragma once
#include <cmath>
namespace custom_dog_control {
// One stream, one update-thread owner. Re-reading one sample is allowed only
// while its original acquisition timestamp remains fresh. A received-at time
// must never be substituted for an unknown hardware acquisition timestamp.
class SampleFreshness {
public:
  bool Accept(double now, double stamp, bool valid, double timeout) {
    if (!valid || !std::isfinite(now) || !std::isfinite(stamp) ||
        !std::isfinite(timeout) || timeout <= 0 || stamp > now ||
        now - stamp >= timeout || (seen_ && (stamp < last_ || now < now_)))
      return false;
    seen_ = true;
    last_ = stamp;
    now_ = now;
    return true;
  }

private:
  bool seen_ = false;
  double last_ = 0, now_ = 0;
};
} // namespace custom_dog_control
