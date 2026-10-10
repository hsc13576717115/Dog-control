#include "custom_dog_control/precision/PrecisionTypes.hpp"
#include <algorithm>
namespace custom_dog_control {
CoordinatedReference::CoordinatedReference() = default;
CoordinatedReference::CoordinatedReference(const CoordinatedReference &other) {
  *this = other;
}
CoordinatedReference &
CoordinatedReference::operator=(const CoordinatedReference &other) {
  // Copy only active knots, bounded even for malformed non-RT requests. Keep
  // the original size so validation still rejects over-capacity input.
  if (this == &other)
    return *this;
  size = other.size;
  lift_index = other.lift_index;
  std::copy_n(other.knots.begin(), std::min(size, capacity), knots.begin());
  return *this;
}
} // namespace custom_dog_control
