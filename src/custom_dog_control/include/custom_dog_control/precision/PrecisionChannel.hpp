#pragma once
#include "custom_dog_control/precision/LatestSnapshot.hpp"
#include "custom_dog_control/precision/PrecisionState.hpp"
#include <atomic>
#include <realtime_tools/realtime_buffer.hpp>
namespace custom_dog_control {
struct PrecisionChannel {
  LatestSnapshot<PrecisionSnapshot> snapshot;
  realtime_tools::RealtimeBuffer<PrecisionCommand> pending;
  std::atomic<bool> active{false}, cancel{false};
};
} // namespace custom_dog_control
