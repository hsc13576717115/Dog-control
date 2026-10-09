#pragma once

#include <mutex>

namespace custom_dog_control {

// Control-to-callback telemetry. Readers copy while locked, never retain a
// pointer into mutable storage. The control thread skips a busy publication;
// consumers must still enforce the sample timestamp's validity period.
// This avoids waiting for callbacks, but does not make T's copy allocation-free.
template <typename T> class LatestSnapshot {
public:
  bool TryWrite(const T &value) {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock())
      return false;
    value_ = value;
    return true;
  }

  T Read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

  // Lifecycle-only blocking reset: call with the control update inactive.
  void Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = T{};
  }

private:
  mutable std::mutex mutex_;
  T value_{};
};

} // namespace custom_dog_control
