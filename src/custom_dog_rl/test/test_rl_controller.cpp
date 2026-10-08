// Controller-level tests use in-memory ros2_control interfaces and synthetic
// IMU messages. They never instantiate an actuator SDK or open a serial device.
#include "custom_dog_rl/RlController.hpp"

#include <gtest/gtest.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace std::chrono_literals;
using custom_dog_rl::RlController;

class ControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    static int sequence = 0;
    name_ = "rl_contract_test_" + std::to_string(++sequence);
    controller_ = std::make_unique<RlController>();
    ASSERT_EQ(controller_->init(name_), controller_interface::return_type::OK);
    peer_ = std::make_shared<rclcpp::Node>(name_ + "_peer");
  }

  void TearDown() override {
    if (active_) controller_->get_node()->deactivate();
    executor_.reset();
    controller_.reset();
    peer_.reset();
  }

  void configure(bool actuation, const std::vector<rclcpp::Parameter>& overrides = {}) {
    auto node = controller_->get_node();
    const auto results = node->set_parameters({
      rclcpp::Parameter("model_path", CUSTOM_DOG_RL_TEST_MODEL),
      rclcpp::Parameter("hardware_mode", "mock"),
      rclcpp::Parameter("enable_actuation", actuation),
      rclcpp::Parameter("imu_topic", "/" + name_ + "/test_imu"),
      rclcpp::Parameter("stand_duration_s", 0.02),
      rclcpp::Parameter("stand_timeout_s", 1.0),
      rclcpp::Parameter("imu_timeout_s", 0.1),
      // Leave realistic inference deadlines enabled: these tests exercise the
      // real worker and model, not a stub action generator.
      rclcpp::Parameter("policy_timeout_s", 0.06),
      rclcpp::Parameter("inference_deadline_s", 0.02),
    });
    for (const auto& result : results) ASSERT_TRUE(result.successful) << result.reason;
    for (const auto& result : node->set_parameters(overrides))
      ASSERT_TRUE(result.successful) << result.reason;
    ASSERT_EQ(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const auto commands = controller_->command_interface_configuration().names;
    const auto states = controller_->state_interface_configuration().names;
    command_handles_.reserve(commands.size());
    state_handles_.reserve(states.size());
    for (const auto& name : commands) {
      const auto slash = name.rfind('/');
      command_values_[name] = 0.0;
      command_handles_.emplace_back(name.substr(0, slash), name.substr(slash + 1), &command_values_.at(name));
    }
    for (const auto& name : states) {
      const auto slash = name.rfind('/');
      state_values_[name] = 0.0;
      state_handles_.emplace_back(name.substr(0, slash), name.substr(slash + 1), &state_values_.at(name));
    }
    for (std::size_t i = 0; i < custom_dog_rl::kActionSize; ++i) {
      const std::string joint = custom_dog_rl::kSdkJointNames[i];
      state_values_[joint + "/position"] = custom_dog_rl::PolicyCore::defaultPositionsSdk()[i];
      state_values_[joint + "/temperature"] = 25.0;
      state_values_[joint + "/valid"] = 1.0;
    }
    state_values_["custom_dog/calibrated"] = 1.0;
    state_values_["custom_dog/communication_ok"] = 1.0;
    std::vector<hardware_interface::LoanedCommandInterface> command_loans;
    std::vector<hardware_interface::LoanedStateInterface> state_loans;
    for (auto& handle : command_handles_) command_loans.emplace_back(handle);
    for (auto& handle : state_handles_) state_loans.emplace_back(handle);
    controller_->assign_interfaces(std::move(command_loans), std::move(state_loans));

    imu_pub_ = peer_->create_publisher<sensor_msgs::msg::Imu>("/" + name_ + "/test_imu", rclcpp::SensorDataQoS());
    mode_pub_ = peer_->create_publisher<std_msgs::msg::String>("/" + name_ + "/mode", rclcpp::QoS(1));
    estop_pub_ = peer_->create_publisher<std_msgs::msg::Bool>("/" + name_ + "/estop", rclcpp::QoS(1).reliable());
    status_sub_ = peer_->create_subscription<std_msgs::msg::String>(
        "/" + name_ + "/status", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::SharedPtr message) { status_ = message->data; });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node->get_node_base_interface());
    executor_->add_node(peer_);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    active_ = true;
    ASSERT_TRUE(until([this] { return status_.find("mode=passive ") != std::string::npos; }, 1s));
  }

  void step(bool publish_imu = true) {
    if (publish_imu) {
      sensor_msgs::msg::Imu message;
      message.header.stamp = peer_->now();
      message.header.frame_id = imu_frame_;
      message.orientation.w = invalid_quaternion_ ? 0.0 : 1.0;
      imu_pub_->publish(message);
    }
    executor_->spin_some();
    EXPECT_EQ(controller_->update(peer_->now(), rclcpp::Duration::from_seconds(0.001)),
              controller_interface::return_type::OK);
    if (emulate_hardware_) {
      const double calibration = command_values_.at("custom_dog/calibrate");
      if (calibration < -0.5) state_values_["custom_dog/calibrated"] = 0.0;
      if (calibration > 0.5) {
        state_values_["custom_dog/calibrated"] = 1.0;
        for (std::size_t i = 0; i < custom_dog_rl::kActionSize; ++i) {
          state_values_[std::string(custom_dog_rl::kSdkJointNames[i]) + "/position"] =
              custom_dog_rl::PolicyCore::defaultPositionsSdk()[i];
        }
      }
      for (const auto* name : custom_dog_rl::kSdkJointNames) {
        const std::string joint = name;
        if (command_values_.at(joint + "/kp") > 0.0)
          state_values_[joint + "/position"] = command_values_.at(joint + "/position");
      }
    }
    executor_->spin_some();
    std::this_thread::sleep_for(1ms);
  }

  template <typename Rep, typename Period>
  bool until(const std::function<bool()>& predicate,
             std::chrono::duration<Rep, Period> timeout, bool publish_imu = true) {
    const auto finish = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < finish) {
      step(publish_imu);
      if (predicate()) return true;
    }
    return predicate();
  }

  void request(const std::string& mode) {
    std_msgs::msg::String message;
    message.data = mode;
    mode_pub_->publish(message);
    // Give DDS delivery a chance before asserting motor commands or transitions.
    for (int i = 0; i < 5; ++i) step();
  }

  bool mode(const char* wanted) const {
    return status_.find(std::string("mode=") + wanted + " ") != std::string::npos;
  }

  void expectNoActuation() const {
    for (const auto* name : custom_dog_rl::kSdkJointNames) {
      const std::string joint = name;
      EXPECT_DOUBLE_EQ(command_values_.at(joint + "/kp"), 0.0) << joint;
      EXPECT_DOUBLE_EQ(command_values_.at(joint + "/kd"), 0.0) << joint;
      EXPECT_DOUBLE_EQ(command_values_.at(joint + "/effort"), 0.0) << joint;
    }
  }

  std::string name_, status_;
  std::string imu_frame_ = "base";
  bool invalid_quaternion_ = false;
  bool active_ = false, emulate_hardware_ = true;
  std::unique_ptr<RlController> controller_;
  rclcpp::Node::SharedPtr peer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::map<std::string, double> command_values_, state_values_;
  std::vector<hardware_interface::CommandInterface> command_handles_;
  std::vector<hardware_interface::StateInterface> state_handles_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
};

TEST_F(ControllerTest, DisabledActuationBlocksStandAndShadowStaysZero) {
  configure(false);
  ASSERT_TRUE(active_);
  request("stand");
  until([] { return false; }, 220ms);
  EXPECT_TRUE(mode("passive")) << status_;
  expectNoActuation();
  request("shadow");
  ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
  expectNoActuation();
  ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms, false)) << status_;
  EXPECT_NE(status_.find("fault=imu_stale_or_invalid"), std::string::npos);
  expectNoActuation();
}

TEST_F(ControllerTest, StandGateRlInferenceAndLatchedEstop) {
  configure(true);
  ASSERT_TRUE(active_);
  request("rl");
  until([] { return false; }, 220ms);
  EXPECT_TRUE(mode("passive")) << status_;
  request("stand");
  ASSERT_TRUE(until([this] { return mode("ready"); }, 900ms)) << status_;
  request("rl");
  ASSERT_TRUE(until([this] { return mode("rl"); }, 500ms)) << status_;
  bool policy_target_differs_from_stand = false;
  for (std::size_t i = 0; i < custom_dog_rl::kActionSize; ++i) {
    const std::string joint = custom_dog_rl::kSdkJointNames[i];
    EXPECT_GT(command_values_.at(joint + "/kp"), 0.0);
    policy_target_differs_from_stand = policy_target_differs_from_stand ||
        std::abs(command_values_.at(joint + "/position") - custom_dog_rl::PolicyCore::defaultPositionsSdk()[i]) > 1e-4;
  }
  EXPECT_TRUE(policy_target_differs_from_stand);
  std_msgs::msg::Bool stop;
  stop.data = true;
  estop_pub_->publish(stop);
  ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms)) << status_;
  EXPECT_NE(status_.find("fault=estop"), std::string::npos);
  EXPECT_DOUBLE_EQ(command_values_.at("custom_dog/emergency_stop"), 1.0);
  for (const auto* joint : custom_dog_rl::kSdkJointNames)
    EXPECT_DOUBLE_EQ(command_values_.at(std::string(joint) + "/kp"), 0.0);
  request("passive");
  EXPECT_TRUE(mode("fault")) << "A still-asserted estop must not be cleared";
  stop.data = false;
  estop_pub_->publish(stop);
  until([] { return false; }, 230ms);
  EXPECT_TRUE(mode("fault")) << "Releasing estop must not restart the policy";
  request("passive");
  ASSERT_TRUE(until([this] { return mode("passive"); }, 500ms)) << status_;
  EXPECT_DOUBLE_EQ(command_values_.at("custom_dog/emergency_stop"), 0.0);
}

TEST_F(ControllerTest, UncalibratedPositionsDoNotBlockExplicitCalibration) {
  configure(false);
  ASSERT_TRUE(active_);
  state_values_["custom_dog/calibrated"] = 0.0;
  state_values_["FR_hip_joint/position"] = 5.0;  // Raw, not yet zeroed encoder.
  request("calibrate");
  ASSERT_TRUE(until([this] { return mode("passive") && state_values_.at("custom_dog/calibrated") > 0.5; }, 500ms)) << status_;
  EXPECT_NEAR(state_values_.at("FR_hip_joint/position"), -0.1, 1e-9);
  expectNoActuation();
  // The same impossible coordinate after calibration must block policy use.
  request("shadow");
  ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
  state_values_["FR_hip_joint/position"] = 5.0;
  ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms)) << status_;
  EXPECT_NE(status_.find("fault=joint_limit"), std::string::npos);
}

TEST_F(ControllerTest, CalibrationLossDuringRlLatchesFault) {
  configure(true);
  ASSERT_TRUE(active_);
  request("stand");
  ASSERT_TRUE(until([this] { return mode("ready"); }, 900ms)) << status_;
  request("rl");
  ASSERT_TRUE(until([this] { return mode("rl"); }, 500ms)) << status_;
  state_values_["custom_dog/calibrated"] = 0.0;
  ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms)) << status_;
  for (const auto* joint : custom_dog_rl::kSdkJointNames)
    EXPECT_DOUBLE_EQ(command_values_.at(std::string(joint) + "/kp"), 0.0);
}

TEST_F(ControllerTest, BriefEstopPulseLatchesBeforeNextControlUpdate) {
  configure(true);
  ASSERT_TRUE(active_);
  request("shadow");
  ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
  std_msgs::msg::Bool message;
  message.data = true;
  estop_pub_->publish(message);
  // Process each message separately, without update(). This exercises a pulse
  // between control cycles, rather than DDS depth-one transport dropping it.
  for (int i = 0; i < 5; ++i) {
    executor_->spin_some();
    std::this_thread::sleep_for(1ms);
  }
  message.data = false;
  estop_pub_->publish(message);
  for (int i = 0; i < 5; ++i) {
    executor_->spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms)) << status_;
  EXPECT_NE(status_.find("fault=estop"), std::string::npos);
  EXPECT_DOUBLE_EQ(command_values_.at("custom_dog/emergency_stop"), 1.0);
  for (const auto* joint : custom_dog_rl::kSdkJointNames)
    EXPECT_DOUBLE_EQ(command_values_.at(std::string(joint) + "/kp"), 0.0);
}

TEST_F(ControllerTest, FeedbackAndControlPeriodFaultsLatch) {
  configure(false);
  ASSERT_TRUE(active_);
  for (int scenario = 0; scenario < 3; ++scenario) {
    request("shadow");
    ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
    std::string expected;
    if (scenario == 0) {
      state_values_["FR_thigh_joint/temperature"] = 80.0;
      expected = "fault=temperature";
    } else if (scenario == 1) {
      state_values_["FR_hip_joint/position"] = std::numeric_limits<double>::quiet_NaN();
      expected = "fault=feedback_invalid";
    } else {
      EXPECT_EQ(controller_->update(peer_->now(), rclcpp::Duration::from_seconds(0.1)),
                controller_interface::return_type::OK);
      expected = "fault=control_period";
    }
    ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms)) << status_;
    EXPECT_NE(status_.find(expected), std::string::npos) << status_;
    expectNoActuation();
    state_values_["FR_thigh_joint/temperature"] = 25.0;
    state_values_["FR_hip_joint/position"] = -0.1;
    request("passive");
    ASSERT_TRUE(until([this] { return mode("passive"); }, 500ms)) << status_;
  }
}

TEST_F(ControllerTest, InvalidImuCannotBeClearedUntilFixed) {
  configure(false);
  ASSERT_TRUE(active_);
  for (int scenario = 0; scenario < 2; ++scenario) {
    request("shadow");
    ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
    if (scenario == 0) imu_frame_ = "wrong_sensor_frame";
    else invalid_quaternion_ = true;
    ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms)) << status_;
    EXPECT_NE(status_.find("fault=imu_stale_or_invalid"), std::string::npos);
    request("passive");
    until([] { return false; }, 220ms);
    EXPECT_TRUE(mode("fault")) << "Invalid orientation must block fault reset";
    expectNoActuation();
    imu_frame_ = "base";
    invalid_quaternion_ = false;
    for (int i = 0; i < 5; ++i) step();
    request("passive");
    ASSERT_TRUE(until([this] { return mode("passive"); }, 500ms)) << status_;
  }
}

TEST_F(ControllerTest, RecalibrationWaitsForClearThenNewZero) {
  configure(false);
  ASSERT_TRUE(active_);
  emulate_hardware_ = false;
  EXPECT_DOUBLE_EQ(state_values_.at("custom_dog/calibrated"), 1.0);
  request("calibrate");
  EXPECT_DOUBLE_EQ(command_values_.at("custom_dog/calibrate"), -1.0);
  ASSERT_TRUE(until([this] { return mode("calibrating"); }, 500ms)) << status_;
  EXPECT_DOUBLE_EQ(command_values_.at("custom_dog/calibrate"), -1.0);
  state_values_["custom_dog/calibrated"] = 0.0;
  step();
  EXPECT_DOUBLE_EQ(command_values_.at("custom_dog/calibrate"), 1.0);
  until([] { return false; }, 220ms);
  EXPECT_TRUE(mode("calibrating")) << "An old calibrated flag cannot complete a new calibration";
  state_values_["custom_dog/calibrated"] = 1.0;
  ASSERT_TRUE(until([this] { return mode("passive"); }, 500ms)) << status_;
  expectNoActuation();
}

TEST_F(ControllerTest, SchedulingPauseFaultsAndNewPolicyEpochCanRestart) {
  configure(false);
  ASSERT_TRUE(active_);
  request("shadow");
  ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
  // Longer than the policy's 60 ms freshness budget, shorter than IMU's
  // 100 ms budget. Resume with fresh IMU and a valid reported loop period.
  std::this_thread::sleep_for(80ms);
  ASSERT_TRUE(until([this] { return mode("fault"); }, 500ms)) << status_;
  EXPECT_NE(status_.find("fault=inference_deadline_or_stale"), std::string::npos) << status_;
  expectNoActuation();
  request("passive");
  ASSERT_TRUE(until([this] { return mode("passive"); }, 500ms)) << status_;
  request("shadow");
  ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
  until([] { return false; }, 150ms);
  EXPECT_TRUE(mode("shadow")) << "Previous-epoch results must not fault a restarted policy";
  expectNoActuation();
}

TEST_F(ControllerTest, EnabledShadowIsZeroImmediatelyAndStandGainsAreSeparate) {
  configure(true, {rclcpp::Parameter("stand_kp", 35.0), rclcpp::Parameter("stand_kd", 1.0)});
  ASSERT_TRUE(active_);
  EXPECT_DOUBLE_EQ(command_values_.at("FR_hip_joint/kd"), 1.0);
  std_msgs::msg::String message;
  message.data = "shadow";
  mode_pub_->publish(message);
  for (int i = 0; i < 5; ++i) {
    executor_->spin_some();
    std::this_thread::sleep_for(1ms);
  }
  // A single transition update must already remove passive damping even when
  // enable_actuation=true; waiting another cycle would hide the regression.
  EXPECT_EQ(controller_->update(peer_->now(), rclcpp::Duration::from_seconds(0.001)),
            controller_interface::return_type::OK);
  expectNoActuation();
  ASSERT_TRUE(until([this] { return mode("shadow"); }, 500ms)) << status_;
  expectNoActuation();
  request("passive");
  ASSERT_TRUE(until([this] { return mode("passive"); }, 500ms)) << status_;
  request("stand");
  ASSERT_TRUE(until([this] { return mode("ready"); }, 900ms)) << status_;
  for (const auto* name : custom_dog_rl::kSdkJointNames) {
    const std::string joint = name;
    EXPECT_DOUBLE_EQ(command_values_.at(joint + "/kp"), 35.0) << joint;
    EXPECT_DOUBLE_EQ(command_values_.at(joint + "/kd"), 1.0) << joint;
  }
  request("rl");
  ASSERT_TRUE(until([this] { return mode("rl"); }, 500ms)) << status_;
  // The emulated joints follow their commanded positions; on a hold tick the
  // effort envelope does not reduce Kp, exposing the configured RL gain.
  ASSERT_TRUE(until([this] {
    for (const auto* name : custom_dog_rl::kSdkJointNames) {
      const std::string joint = name;
      if (command_values_.at(joint + "/kp") != 25.0 || command_values_.at(joint + "/kd") != 0.5)
        return false;
    }
    return true;
  }, 100ms)) << "RL must retain training gains when stand gains are overridden";
}

TEST_F(ControllerTest, WrongModelChecksumPreventsConfiguration) {
  auto node = controller_->get_node();
  node->set_parameter(rclcpp::Parameter("model_path", CUSTOM_DOG_RL_TEST_MODEL));
  node->set_parameter(rclcpp::Parameter("expected_model_sha256", std::string(64, '0')));
  EXPECT_NE(controller_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}
}  // namespace
