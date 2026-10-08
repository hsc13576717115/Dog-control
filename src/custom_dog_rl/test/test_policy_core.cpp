#include "custom_dog_rl/PolicyCore.hpp"

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace custom_dog_rl;

void check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message, double tolerance = 1e-6) {
  check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

void rejects(const std::function<void()>& operation, const char* message) {
  try {
    operation();
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error(message);
}

SensorFrame standing() {
  SensorFrame sensors;
  sensors.position_sdk = PolicyCore::defaultPositionsSdk();
  return sensors;
}

void testStandingAndReset() {
  PolicyCore core;
  const auto sensors = standing();
  const Observation first = core.observe(sensors, {0.2, -0.3, 0.4});
  near(first[0], 0.2, "vx must already be included in the 45D current frame");
  near(first[1], -0.3, "vy command order");
  near(first[2], 0.4, "yaw command order");
  near(first[8], -1.0, "upright gravity points down");
  for (std::size_t i = 9; i < first.size(); ++i) {
    near(first[i], 0.0, "standing offsets, previous action and reset history must be zero");
  }
  Action action{};
  action.fill(2.0F);
  core.acceptAction(action);
  core.observe(sensors, {});
  core.reset();
  const auto after_reset = core.observe(sensors, {0.2, -0.3, 0.4});
  check(first == after_reset, "reset must remove action and all history from previous episode");
}

void testHistoryAndRawAction() {
  PolicyCore core;
  const auto sensors = standing();
  for (int frame = 1; frame <= 8; ++frame) {
    Action previous{};
    previous.fill(static_cast<float>(frame - 1));
    core.acceptAction(previous);
    core.observe(sensors, {static_cast<double>(frame), 0.0, 0.0});
  }
  // The ninth observation leaves exactly frames 9,8,7,6,5,4.
  const auto history = core.observe(sensors, {9.0, 0.0, 0.0});
  for (std::size_t frame = 0; frame < 6; ++frame) {
    near(history[frame * 45], 9.0 - frame, "newest-first history or oldest-frame eviction is wrong");
  }
  near(history[33], 7.0, "last_action must use last accepted raw output, without decoding");
  near(history[45 + 33], 7.0, "history must store the action available at its original timestep");
  near(history[2 * 45 + 33], 6.0, "older actions must not be overwritten retroactively");
}

void testJointMapping() {
  PolicyCore core;
  auto sensors = standing();
  for (std::size_t sdk = 0; sdk < 12; ++sdk) {
    sensors.position_sdk[sdk] += static_cast<double>(sdk + 1);
    sensors.velocity_sdk[sdk] = static_cast<double>(sdk + 1) * 2.0;
  }
  const auto observation = core.observe(sensors, {});
  const double expected_offsets[] = {4, 1, 10, 7, 5, 2, 11, 8, 6, 3, 12, 9};
  for (std::size_t policy = 0; policy < 12; ++policy) {
    near(observation[9 + policy], expected_offsets[policy], "SDK-to-policy joint position mapping");
    near(observation[21 + policy], 0.1 * expected_offsets[policy], "joint velocity mapping or scale");
  }
  Action action{};
  for (std::size_t policy = 0; policy < 12; ++policy) {
    action[policy] = static_cast<float>(policy + 1);
  }
  const auto target = core.decode(action);
  const double expected_sdk[] = {0.4, 2.3, 1.0, 0.35, 2.05, 0.75,
                                 0.9, 2.8, 1.5, 0.85, 2.55, 1.25};
  for (std::size_t sdk = 0; sdk < 12; ++sdk) {
    near(target[sdk], expected_sdk[sdk], "policy-to-SDK action mapping or default pose");
  }
}

void testClippingOrder() {
  PolicyCore core;
  auto sensors = standing();
  sensors.angular_velocity_body = {120.0, -120.0, 8.0};
  sensors.velocity_sdk.fill(200.0);
  sensors.position_sdk.fill(200.0);
  Action raw{};
  raw.fill(400.0F);
  core.acceptAction(raw);
  const auto observation = core.observe(sensors, {120.0, -120.0, 2.0});
  near(observation[0], 100.0, "command clip");
  near(observation[1], -100.0, "negative command clip");
  near(observation[3], 25.0, "angular velocity must clip before .25 scaling");
  near(observation[4], -25.0, "negative angular velocity scale");
  near(observation[5], 2.0, "angular velocity scale");
  near(observation[9], 100.0, "relative position clip");
  near(observation[21], 5.0, "joint velocity must clip before .05 scaling");
  near(observation[33], 100.0, "raw previous action observation clip");
  const auto target = core.decode(raw);
  near(target[3], 100.0, "target must be affine then clip, never clip raw action first");
  near(target[2], 98.5, "calf default pose must be added before target clipping");
}

void testGravityFrameAndNormalization() {
  PolicyCore core;
  auto sensors = standing();
  const double half = std::sqrt(0.5);
  sensors.orientation_wxyz = {half, half, 0.0, 0.0};
  auto observation = core.observe(sensors, {});
  near(observation[6], 0.0, "roll gravity x");
  near(observation[7], -1.0, "base-to-world quaternion must be inverted for gravity");
  near(observation[8], 0.0, "roll gravity z");
  sensors.orientation_wxyz = {2.0 * half, 0.0, 2.0 * half, 0.0};
  observation = core.observe(sensors, {});
  near(observation[6], 1.0, "pitch gravity x and quaternion normalization");
  near(observation[7], 0.0, "pitch gravity y");
  near(observation[8], 0.0, "pitch gravity z");
  sensors.orientation_wxyz = {-half, 0.0, 0.0, -half};
  observation = core.observe(sensors, {});
  near(observation[6], 0.0, "yaw must not affect projected gravity x");
  near(observation[7], 0.0, "yaw must not affect projected gravity y");
  near(observation[8], -1.0, "quaternion sign must not change gravity");
}

void testInvalidDataDoesNotAdvanceState() {
  PolicyCore core;
  const auto valid = standing();
  const auto first = core.observe(valid, {1.0, 0.0, 0.0});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  rejects([&] { core.observe(valid, {nan, 0.0, 0.0}); }, "NaN command accepted");
  auto invalid = valid;
  invalid.position_sdk[2] = inf;
  rejects([&] { core.observe(invalid, {}); }, "infinite joint position accepted");
  invalid = valid;
  invalid.velocity_sdk[1] = nan;
  rejects([&] { core.observe(invalid, {}); }, "NaN joint velocity accepted");
  invalid = valid;
  invalid.angular_velocity_body[0] = inf;
  rejects([&] { core.observe(invalid, {}); }, "infinite gyro accepted");
  invalid = valid;
  invalid.orientation_wxyz.fill(0.0);
  rejects([&] { core.observe(invalid, {}); }, "zero quaternion accepted");
  invalid.orientation_wxyz[0] = nan;
  rejects([&] { core.observe(invalid, {}); }, "NaN quaternion accepted");
  Action bad_action{};
  bad_action[11] = std::numeric_limits<float>::quiet_NaN();
  rejects([&] { core.acceptAction(bad_action); }, "NaN action accepted into history");
  rejects([&] { core.decode(bad_action); }, "NaN decoded action accepted");
  const auto second = core.observe(valid, {2.0, 0.0, 0.0});
  for (std::size_t i = 0; i < 45; ++i) {
    near(second[45 + i], first[i], "rejected sensor data must not advance history");
  }
  near(second[33 + 11], 0.0, "rejected action must not replace previous action");
}
}  // namespace

int main() {
  try {
    testStandingAndReset();
    testHistoryAndRawAction();
    testJointMapping();
    testClippingOrder();
    testGravityFrameAndNormalization();
    testInvalidDataDoesNotAdvanceState();
    std::cout << "PASS: policy contract, history, mapping, clipping, gravity and invalid inputs\n";
    return EXIT_SUCCESS;
  } catch (const std::exception& exception) {
    std::cerr << "FAIL: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
}
