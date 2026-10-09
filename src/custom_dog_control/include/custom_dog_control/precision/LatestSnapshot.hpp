#pragma once

#include <array>
#include <atomic>
#include <mutex>

namespace custom_dog_control {

// Single producer, serialized non-RT consumers. Each side owns one slot; an
// atomic exchange transfers ownership of the third. No retry loop or reader
// mutex on Publish. Intermediate samples may be superseded. T must be a
// bounded, allocation-free value in the control path (PrecisionSnapshot has
// fixed fields). This only bounds the exchange; it does not establish a hard-RT
// WBC deadline.
template <typename T> class LatestSnapshot {
public:
  static_assert(std::atomic<unsigned>::is_always_lock_free,
                "snapshot ownership exchange requires lock-free atomics");
  void Publish(const T &value) {
    slots_[back_] = value;
    back_ =
        middle_.exchange(back_ | kDirty, std::memory_order_acq_rel) & kIndex;
  }

  T Read() const {
    std::lock_guard<std::mutex> lock(reader_mutex_);
    if (middle_.load(std::memory_order_acquire) & kDirty)
      front_ = middle_.exchange(front_, std::memory_order_acq_rel) & kIndex;
    return slots_[front_];
  }

  // Lifecycle only: producer must be stopped; readers are serialized here.
  void Reset() {
    std::lock_guard<std::mutex> lock(reader_mutex_);
    for (auto &slot : slots_)
      slot = T{};
    front_ = 0;
    back_ = 2;
    middle_.store(1, std::memory_order_release);
  }

private:
  static constexpr unsigned kDirty = 4, kIndex = 3;
  std::array<T, 3> slots_{};
  unsigned back_ = 2;
  mutable unsigned front_ = 0;
  mutable std::atomic<unsigned> middle_{1};
  mutable std::mutex reader_mutex_;
};

} // namespace custom_dog_control
