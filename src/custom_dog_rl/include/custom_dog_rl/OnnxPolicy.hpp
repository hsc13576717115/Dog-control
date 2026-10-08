#pragma once

#include <memory>
#include <string>

#include "custom_dog_rl/PolicyCore.hpp"

namespace custom_dog_rl {

// CPU FP32 runtime for policy_combined.onnx. It owns reusable input/output
// tensors and validates exact names, dtype and dimensions before first use.
// One instance belongs to one worker; concurrent infer calls are unsupported.
// The public header intentionally has no ONNX Runtime dependency.
class OnnxPolicy {
 public:
  explicit OnnxPolicy(const std::string& path, int threads = 1);
  ~OnnxPolicy();
  OnnxPolicy(const OnnxPolicy&) = delete;
  OnnxPolicy& operator=(const OnnxPolicy&) = delete;

  Action infer(const Observation& observation);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace custom_dog_rl
