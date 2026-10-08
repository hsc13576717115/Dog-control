#include "custom_dog_rl/OnnxPolicy.hpp"

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void rejects(const std::function<void()>& operation, const char* message) {
  try {
    operation();
  } catch (const std::exception&) {
    return;
  }
  throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
  using namespace custom_dog_rl;
  try {
    check(argc == 2, "Usage: test_onnx_policy <policy_combined.onnx>");
    rejects([] { OnnxPolicy policy(""); }, "Empty model path accepted");
    rejects([&] { OnnxPolicy policy(argv[1], 0); }, "Zero runtime threads accepted");
    rejects([&] { OnnxPolicy policy(argv[1], -1); }, "Negative runtime threads accepted");
    OnnxPolicy policy(argv[1], 1);
    Observation observation{};
    const auto first = policy.infer(observation);
    for (const auto value : first) check(std::isfinite(value), "Nonfinite initial action");
    observation[0] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { policy.infer(observation); }, "NaN observation accepted");
    observation[0] = std::numeric_limits<float>::infinity();
    rejects([&] { policy.infer(observation); }, "Infinite observation accepted");
    observation[0] = 0.0F;
    const auto after_rejection = policy.infer(observation);
    check(first == after_rejection, "Failed inference must not poison the next valid result");
    for (int i = 0; i < 100; ++i) {
      check(policy.infer(observation) == first, "Identical observations produced inconsistent actions");
    }
    std::cout << "PASS: ONNX finite inference, reusable tensors, invalid inputs and configuration\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& exception) {
    std::cerr << "FAIL: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
}
