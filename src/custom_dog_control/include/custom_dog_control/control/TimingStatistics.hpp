#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace custom_dog_control {

struct TimingSnapshot {
  double mean = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double maximum = 0.0;
  std::size_t samples = 0;
};

template <std::size_t Capacity>
class TimingWindow {
 public:
  static_assert(Capacity > 0, "TimingWindow capacity must be positive");

  void Add(double value) {
    values_[next_] = value;
    next_ = (next_ + 1) % Capacity;
    count_ = std::min(count_ + 1, Capacity);
  }

  void Reset() {
    next_ = 0;
    count_ = 0;
  }

  TimingSnapshot Snapshot() const {
    TimingSnapshot result;
    result.samples = count_;
    if (count_ == 0) {
      return result;
    }
    std::array<double, Capacity> sorted{};
    double sum = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
      sorted[i] = values_[i];
      sum += values_[i];
    }
    std::sort(sorted.begin(), sorted.begin() + count_);
    result.mean = sum / static_cast<double>(count_);
    result.p95 = sorted[PercentileIndex(0.95)];
    result.p99 = sorted[PercentileIndex(0.99)];
    result.maximum = sorted[count_ - 1];
    return result;
  }

 private:
  std::size_t PercentileIndex(double percentile) const {
    const auto nearest_rank = static_cast<std::size_t>(
        std::ceil(percentile * static_cast<double>(count_)));
    return std::min(std::max(nearest_rank, std::size_t{1}) - 1, count_ - 1);
  }

  std::array<double, Capacity> values_{};
  std::size_t next_ = 0;
  std::size_t count_ = 0;
};

}  // namespace custom_dog_control
