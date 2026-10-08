#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "custom_dog_rl/OnnxPolicy.hpp"
#include "custom_dog_rl/PolicyCore.hpp"

namespace custom_dog_rl {

// The hardware loop never calls ONNX. A single worker owns the policy and history;
// epoch tags invalidate in-flight work when leaving RL or resetting a fault.
class RlController final : public controller_interface::ControllerInterface {
 public:
  ~RlController() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

 private:
  enum class Mode : int { Passive, Calibrating, Standing, Ready, Rl, Shadow, Fault };
  enum class Request : int { None, Passive, Calibrate, Stand, Rl, Shadow, Estop };
  enum class Fault : int { None, Imu, Feedback, Limits, Tilt, Temperature, Effort,
    Velocity, Deadline, Policy, StandTimeout, Estop, Period, CalibrationTimeout };
  struct ImuInput {
    std::array<double, 4> orientation{1, 0, 0, 0};
    std::array<double, 3> gyro{};
    double received = 0;
    bool valid = false;
  };
  struct CommandInput { std::array<double, 3> velocity{}; double received = 0; };
  struct Work {
    SensorFrame sensor;
    std::array<double, 3> command{};
    Action previous_action{};
    std::uint64_t epoch = 0, sequence = 0;
    double observed = 0;
  };
  struct Result {
    Action action{};
    std::uint64_t epoch = 0, sequence = 0;
    double observed = 0, duration = 0;
    bool valid = false;
  };
  static double nowSteady();
  static const char* modeName(Mode);
  static const char* faultName(Fault);
  bool hybrid() const { return hardware_mode_ != "gazebo"; }
  void stopWorker();
  void workerLoop();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr);
  bool resolveInterfaces();
  Fault readSensors(SensorFrame&, const ImuInput&, double now, bool need_imu);
  void transition(Mode, double now);
  void latch(Fault, double now);
  void writePassive();
  void writeTarget(const JointArray&, const SensorFrame&, double dt);

  std::string hardware_mode_, imu_frame_;
  bool enable_actuation_ = false;
  double imu_timeout_ = .1, command_timeout_ = .3, policy_timeout_ = .06;
  double deadline_ = .02, stand_duration_ = 3, stand_timeout_ = 8;
  double calibration_timeout_ = 3, stand_tolerance_ = .15, stand_velocity_ = .5;
  double safe_damping_ = 1, kp_ = 25, kd_ = .5, max_tilt_ = .9;
  double stand_kp_ = 25, stand_kd_ = .5;
  double max_temperature_ = 70, joint_tolerance_ = .05, target_margin_ = .03;
  double target_rate_ = 0, max_control_period_ = .02;
  std::array<double, 4> imu_to_body_{1, 0, 0, 0};
  JointArray lower_{}, upper_{}, effort_limit_{}, velocity_limit_{};
  JointArray stand_start_{}, target_{};
  std::array<std::array<std::size_t, 5>, 12> command_indices_{}, state_indices_{};
  std::array<std::size_t, 2> system_commands_{};
  std::array<std::size_t, 3> system_states_{};
  bool calibrated_ = false, calibration_zero_seen_ = false;
  Mode mode_ = Mode::Passive;
  Fault fault_ = Fault::None;
  double entered_ = 0, next_inference_ = 0, settled_since_ = 0;
  std::uint64_t epoch_ = 0, submitted_ = 0, accepted_ = 0;
  Action previous_action_{};
  std::atomic<Request> request_{Request::None};
  std::atomic<bool> external_estop_{false};
  std::atomic<int> published_mode_{0}, published_fault_{0};
  std::atomic<double> inference_duration_{0}, inference_observed_{0};
  std::atomic<std::uint64_t> inference_count_{0};
  realtime_tools::RealtimeBuffer<ImuInput> imu_buffer_;
  realtime_tools::RealtimeBuffer<CommandInput> command_buffer_;
  realtime_tools::RealtimeBuffer<Result> result_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::int64_t last_imu_stamp_ = 0;
  std::unique_ptr<OnnxPolicy> policy_;
  std::thread worker_;
  std::mutex work_mutex_;
  std::condition_variable work_ready_;
  Work pending_;
  bool has_work_ = false, stopping_ = false;
};
}  // namespace custom_dog_rl
