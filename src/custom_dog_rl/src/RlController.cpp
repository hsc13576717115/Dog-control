#include "custom_dog_rl/RlController.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <openssl/evp.h>
#include <pluginlib/class_list_macros.hpp>

namespace custom_dog_rl {
namespace {
using Quaternion = std::array<double, 4>;
Quaternion multiply(const Quaternion& a, const Quaternion& b) {
  return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
    a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
    a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
    a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
Quaternion conjugate(Quaternion q) { q[1]*=-1; q[2]*=-1; q[3]*=-1; return q; }
bool normalize(Quaternion& q) {
  double norm = 0;
  for (double x : q) { if (!std::isfinite(x)) return false; norm += x*x; }
  if (norm < .25 || norm > 2.25) return false;
  norm = std::sqrt(norm);
  for (double& x : q) x /= norm;
  return true;
}
std::string sha256(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read ONNX file: " + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  std::array<char, 65536> block{};
  while (file.read(block.data(), block.size()) || file.gcount()) {
    if (EVP_DigestUpdate(ctx.get(), block.data(), file.gcount()) != 1)
      throw std::runtime_error("SHA256 update failed");
  }
  if (!file.eof()) throw std::runtime_error("ONNX read failed");
  unsigned char digest[EVP_MAX_MD_SIZE]; unsigned int count = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest, &count) != 1)
    throw std::runtime_error("SHA256 finalization failed");
  std::ostringstream out;
  for (unsigned int i=0; i<count; ++i)
    out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
  return out.str();
}
constexpr std::array<const char*, 5> kCommandFields{"position", "velocity", "effort", "kp", "kd"};
constexpr std::array<const char*, 5> kStateFields{"position", "velocity", "effort", "temperature", "valid"};
std::vector<double> repeated(double hip, double thigh, double calf) {
  return {hip, thigh, calf, hip, thigh, calf, hip, thigh, calf, hip, thigh, calf};
}
}  // namespace

RlController::~RlController() { stopWorker(); }
double RlController::nowSteady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
const char* RlController::modeName(Mode mode) {
  static const char* names[]{"passive", "calibrating", "standing", "ready", "rl", "shadow", "fault"};
  return names[static_cast<int>(mode)];
}
const char* RlController::faultName(Fault fault) {
  static const char* names[]{"none", "imu_stale_or_invalid", "feedback_invalid", "joint_limit",
    "tilt", "temperature", "effort", "joint_velocity", "inference_deadline_or_stale",
    "policy_invalid", "stand_timeout", "estop", "control_period", "calibration_timeout"};
  return names[static_cast<int>(fault)];
}
controller_interface::CallbackReturn RlController::on_init() {
  try {
    auto_declare<std::string>("hardware_mode", "mock");
    auto_declare<std::string>("model_path", "");
    auto_declare<std::string>("expected_model_sha256", "10bcd8da253aba3e21dad7a417a73258a5753b30bc04ef63433ec706852fa54c");
    auto_declare<std::string>("imu_topic", "/imu");
    auto_declare<std::string>("imu_frame", "base");
    auto_declare<bool>("enable_actuation", false);
    auto_declare<int>("inference_threads", 1);
    auto_declare<std::vector<double>>("imu_to_body_wxyz", {1,0,0,0});
    auto_declare<std::vector<double>>("joint_lower_sdk", repeated(-1.0472,-1.5708,-2.8274333882));
    auto_declare<std::vector<double>>("joint_upper_sdk", repeated(1.0472,3.4907,-.83775804096));
    auto_declare<std::vector<double>>("effort_limits_sdk", repeated(23.7,23.7,45));
    auto_declare<std::vector<double>>("velocity_limits_sdk", repeated(30,30,15));
    for (auto entry : std::vector<std::pair<std::string,double>>{
      {"imu_timeout_s",.1},{"command_timeout_s",.3},{"policy_timeout_s",.06},
      {"inference_deadline_s",.02},{"stand_duration_s",3},{"stand_timeout_s",8},
      {"calibration_timeout_s",3},{"stand_tolerance_rad",.15},{"stand_velocity_rad_s",.5},
      {"safe_damping",1},{"kp",25},{"kd",.5},{"stand_kp",25},{"stand_kd",.5},{"max_tilt_rad",.9},
      {"max_temperature_c",70},{"joint_limit_tolerance_rad",.05},
      {"target_limit_margin_rad",.03},{"target_rate_limit_rad_s",0},{"max_control_period_s",.02}})
      auto_declare<double>(entry.first, entry.second);
    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}
controller_interface::InterfaceConfiguration RlController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto* joint : kSdkJointNames) {
    if (hybrid()) for (auto field : kCommandFields) c.names.push_back(std::string(joint)+"/"+field);
    else c.names.push_back(std::string(joint)+"/effort");
  }
  if (hybrid()) for (auto field : {"calibrate","emergency_stop"}) c.names.push_back(std::string("custom_dog/")+field);
  return c;
}
controller_interface::InterfaceConfiguration RlController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto* joint : kSdkJointNames)
    for (std::size_t j=0; j<(hybrid()?5u:3u); ++j) c.names.push_back(std::string(joint)+"/"+kStateFields[j]);
  if (hybrid()) for (auto field : {"calibrated","communication_ok","physical_estop"}) c.names.push_back(std::string("custom_dog/")+field);
  return c;
}
controller_interface::CallbackReturn RlController::on_configure(const rclcpp_lifecycle::State&) {
  try {
    stopWorker();
    auto n = get_node();
    hardware_mode_ = n->get_parameter("hardware_mode").as_string();
    if (hardware_mode_!="real" && hardware_mode_!="mock" && hardware_mode_!="gazebo")
      throw std::invalid_argument("hardware_mode must be real, mock or gazebo");
    enable_actuation_ = n->get_parameter("enable_actuation").as_bool();
    imu_frame_ = n->get_parameter("imu_frame").as_string();
    if(imu_frame_.empty()) throw std::invalid_argument("imu_frame must identify the calibrated sensor frame");
    auto mount=n->get_parameter("imu_to_body_wxyz").as_double_array();
    if (mount.size()!=4) throw std::invalid_argument("imu_to_body_wxyz must have four values");
    std::copy(mount.begin(),mount.end(),imu_to_body_.begin());
    if (!normalize(imu_to_body_)) throw std::invalid_argument("Invalid IMU mounting quaternion");
    auto array_param=[&](const char* name, JointArray& target) {
      auto values=n->get_parameter(name).as_double_array();
      if (values.size()!=12) throw std::invalid_argument(std::string(name)+" requires twelve values");
      for (double value : values) if (!std::isfinite(value)) throw std::invalid_argument("Non-finite joint parameter");
      std::copy(values.begin(),values.end(),target.begin());
    };
    array_param("joint_lower_sdk",lower_); array_param("joint_upper_sdk",upper_);
    array_param("effort_limits_sdk",effort_limit_); array_param("velocity_limits_sdk",velocity_limit_);
    for (auto entry : std::vector<std::pair<const char*,double*>>{
      {"imu_timeout_s",&imu_timeout_},{"command_timeout_s",&command_timeout_},
      {"policy_timeout_s",&policy_timeout_},{"inference_deadline_s",&deadline_},
      {"stand_duration_s",&stand_duration_},{"stand_timeout_s",&stand_timeout_},
      {"calibration_timeout_s",&calibration_timeout_},{"stand_tolerance_rad",&stand_tolerance_},
      {"stand_velocity_rad_s",&stand_velocity_},{"safe_damping",&safe_damping_},
      {"kp",&kp_},{"kd",&kd_},{"stand_kp",&stand_kp_},{"stand_kd",&stand_kd_},{"max_tilt_rad",&max_tilt_},
      {"max_temperature_c",&max_temperature_},{"joint_limit_tolerance_rad",&joint_tolerance_},
      {"target_limit_margin_rad",&target_margin_},{"target_rate_limit_rad_s",&target_rate_},
      {"max_control_period_s",&max_control_period_}}) {
      *entry.second=n->get_parameter(entry.first).as_double();
      if (!std::isfinite(*entry.second) || *entry.second<0)
        throw std::invalid_argument(std::string("Invalid parameter ")+entry.first);
    }
    if (imu_timeout_<=0 || command_timeout_<=0 || policy_timeout_<.02 || deadline_<=0 ||
        deadline_>policy_timeout_ || stand_duration_<=0 || stand_timeout_<=stand_duration_ ||
        calibration_timeout_<=0 || stand_tolerance_<=0 || stand_velocity_<=0 || kp_<=0 || kd_<=0 ||
        stand_kp_<=0 || stand_kd_<=0 || max_tilt_<=0 || max_tilt_>=1.57 || max_temperature_<=0 || max_control_period_<=0)
      throw std::invalid_argument("Inconsistent timing, gain or safety parameters");
    for (std::size_t i=0;i<12;++i) {
      if (lower_[i]+2*target_margin_>=upper_[i] || effort_limit_[i]<=0 || velocity_limit_[i]<=0 ||
          PolicyCore::defaultPositionsSdk()[i]<lower_[i]+target_margin_ ||
          PolicyCore::defaultPositionsSdk()[i]>upper_[i]-target_margin_)
        throw std::invalid_argument("Invalid joint bounds or nominal pose");
    }
    // Matching tensor sizes alone cannot verify observation order or actuator mapping.
    const auto model=n->get_parameter("model_path").as_string();
    const auto expected=n->get_parameter("expected_model_sha256").as_string();
    if (expected.size()!=64 || sha256(model)!=expected)
      throw std::invalid_argument("Model SHA256 mismatch; use the verified deployment contract");
    const auto threads=n->get_parameter("inference_threads").as_int();
    if (threads<1 || threads>32) throw std::invalid_argument("inference_threads must be 1..32");
    policy_=std::make_unique<OnnxPolicy>(model,static_cast<int>(threads));
    // Page in the model and initialize kernels before entering the timed loop.
    for(int warmup=0;warmup<3;++warmup) policy_->infer(Observation{});
    last_imu_stamp_=0;
    inference_duration_.store(0); inference_observed_.store(0); inference_count_.store(0);
    imu_buffer_.initRT(ImuInput{}); command_buffer_.initRT(CommandInput{}); result_buffer_.initRT(Result{});
    request_.store(Request::None); external_estop_.store(false);
    imu_sub_=n->create_subscription<sensor_msgs::msg::Imu>(n->get_parameter("imu_topic").as_string(),
      rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::Imu::SharedPtr m){imuCallback(m);});
    command_sub_=n->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel",rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::SharedPtr m){
        CommandInput input{{m->linear.x,m->linear.y,m->angular.z},nowSteady()};
        for(double x:input.velocity) if(!std::isfinite(x)) { command_buffer_.writeFromNonRT(CommandInput{}); return; }
        input.velocity[0]=std::clamp(input.velocity[0],-1.,1.);
        input.velocity[1]=std::clamp(input.velocity[1],-1.,1.);
        input.velocity[2]=std::clamp(input.velocity[2],-2.,2.);
        command_buffer_.writeFromNonRT(input);
      });
    mode_sub_=n->create_subscription<std_msgs::msg::String>("~/mode",rclcpp::QoS(1),
      [this](std_msgs::msg::String::SharedPtr m){
        Request requested=Request::None;
        if(m->data=="passive") requested=Request::Passive;
        else if(m->data=="calibrate") requested=Request::Calibrate;
        else if(m->data=="stand") requested=Request::Stand;
        else if(m->data=="rl") requested=Request::Rl;
        else if(m->data=="shadow") requested=Request::Shadow;
        else if(m->data=="estop") requested=Request::Estop;
        if (requested==Request::Estop || request_.load()!=Request::Estop) request_.store(requested);
      });
    estop_sub_=n->create_subscription<std_msgs::msg::Bool>("~/estop",rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Bool::SharedPtr m){
        external_estop_.store(m->data);
        if(m->data) request_.store(Request::Estop);
      });
    status_pub_=n->create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).transient_local());
    status_timer_=n->create_wall_timer(std::chrono::milliseconds(200),[this]{
      std_msgs::msg::String status;
      status.data=std::string("mode=")+modeName(static_cast<Mode>(published_mode_.load()))+
        " fault="+faultName(static_cast<Fault>(published_fault_.load()))+
        " actuation="+(enable_actuation_?"enabled":"disabled")+
        " inference_ms="+std::to_string(1000*inference_duration_.load())+
        " observation_age_ms="+std::to_string(inference_observed_.load()>0?1000*(nowSteady()-inference_observed_.load()):-1)+
        " inference_count="+std::to_string(inference_count_.load());
      status_pub_->publish(status);
    });
    return controller_interface::CallbackReturn::SUCCESS;
  } catch(const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(),"RL configure failed: %s",e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}
void RlController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr m) {
  ImuInput input;
  if(m->header.stamp.sec<0 || m->header.stamp.nanosec>=1000000000u) {
    imu_buffer_.writeFromNonRT(input);
    return;
  }
  const rclcpp::Time stamp(m->header.stamp,get_node()->get_clock()->get_clock_type());
  const double age=(get_node()->now()-stamp).seconds();
  const auto ns=stamp.nanoseconds();
  Quaternion world_from_imu{m->orientation.w,m->orientation.x,m->orientation.y,m->orientation.z};
  const bool valid=m->header.frame_id==imu_frame_ && ns>0 && ns>last_imu_stamp_ &&
    age>=-.02 && age<=imu_timeout_ && m->orientation_covariance[0]!=-1 &&
    m->angular_velocity_covariance[0]!=-1 && normalize(world_from_imu) &&
    std::isfinite(m->angular_velocity.x) && std::isfinite(m->angular_velocity.y) && std::isfinite(m->angular_velocity.z);
  if(valid) {
    last_imu_stamp_=ns;
    input.orientation=multiply(world_from_imu,conjugate(imu_to_body_));
    const auto rotated=multiply(multiply(imu_to_body_,Quaternion{0,m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z}),conjugate(imu_to_body_));
    input.gyro={rotated[1],rotated[2],rotated[3]}; input.received=nowSteady(); input.valid=true;
  }
  // Invalid or repeated messages do not refresh the watchdog with old data.
  imu_buffer_.writeFromNonRT(input);
}
bool RlController::resolveInterfaces() {
  auto lookup=[](const auto& interfaces,const std::string& name,std::size_t& index) {
    for(std::size_t i=0;i<interfaces.size();++i) {
      if(interfaces[i].get_name()==name) {index=i;return true;}
    }
    return false;
  };
  for(std::size_t i=0;i<12;++i) {
    for(std::size_t j=0;j<(hybrid()?5u:3u);++j)
      if(!lookup(state_interfaces_,std::string(kSdkJointNames[i])+"/"+kStateFields[j],state_indices_[i][j])) return false;
    if(hybrid()) {
      for(std::size_t j=0;j<5;++j) if(!lookup(command_interfaces_,std::string(kSdkJointNames[i])+"/"+kCommandFields[j],command_indices_[i][j])) return false;
    } else if(!lookup(command_interfaces_,std::string(kSdkJointNames[i])+"/effort",command_indices_[i][2])) return false;
  }
  if(hybrid()) {
    for(std::size_t i=0;i<2;++i) if(!lookup(command_interfaces_,std::string("custom_dog/")+(i==0?"calibrate":"emergency_stop"),system_commands_[i])) return false;
    const char* fields[]{"calibrated","communication_ok","physical_estop"};
    for(std::size_t i=0;i<3;++i) if(!lookup(state_interfaces_,std::string("custom_dog/")+fields[i],system_states_[i])) return false;
  }
  return true;
}
controller_interface::CallbackReturn RlController::on_activate(const rclcpp_lifecycle::State&) {
  if(!policy_ || !resolveInterfaces()) return controller_interface::CallbackReturn::ERROR;
  request_.store(Request::None); fault_=Fault::None;
  transition(Mode::Passive,nowSteady());
  {std::lock_guard<std::mutex> lock(work_mutex_); stopping_=false; has_work_=false;}
  worker_=std::thread(&RlController::workerLoop,this);
  writePassive();
  return controller_interface::CallbackReturn::SUCCESS;
}
void RlController::stopWorker() {
  {std::lock_guard<std::mutex> lock(work_mutex_); stopping_=true; has_work_=false;}
  work_ready_.notify_one();
  if(worker_.joinable()) worker_.join();
}
controller_interface::CallbackReturn RlController::on_deactivate(const rclcpp_lifecycle::State&) {
  transition(Mode::Passive,nowSteady()); writePassive(); stopWorker();
  return controller_interface::CallbackReturn::SUCCESS;
}
void RlController::workerLoop() {
  PolicyCore core;
  std::uint64_t worker_epoch=0;
  for(;;) {
    Work work;
    {std::unique_lock<std::mutex> lock(work_mutex_);
      work_ready_.wait(lock,[this]{return stopping_||has_work_;});
      if(stopping_) return;
      work=pending_; has_work_=false;
    }
    Result result; result.epoch=work.epoch; result.sequence=work.sequence; result.observed=work.observed;
    const double start=nowSteady();
    try {
      if(worker_epoch!=work.epoch) {core.reset();worker_epoch=work.epoch;}
      core.acceptAction(work.previous_action);
      result.action=policy_->infer(core.observe(work.sensor,work.command));
      result.valid=true;
      for(float x:result.action) if(!std::isfinite(x)) result.valid=false;
    } catch(const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(),"Inference failed: %s",e.what());
    }
    result.duration=nowSteady()-start;
    inference_duration_.store(result.duration); inference_observed_.store(work.observed); ++inference_count_;
    result_buffer_.writeFromNonRT(result);
  }
}
RlController::Fault RlController::readSensors(SensorFrame& sensor,const ImuInput& imu,double now,bool need_imu) {
  calibrated_=!hybrid();
  if(hybrid()) {
    const double calibrated=state_interfaces_[system_states_[0]].get_value();
    const double communication=state_interfaces_[system_states_[1]].get_value();
    const double estop=state_interfaces_[system_states_[2]].get_value();
    if(!std::isfinite(estop) || estop>.5 || external_estop_.load()) return Fault::Estop;
    if(!std::isfinite(communication) || communication<.5 || !std::isfinite(calibrated)) return Fault::Feedback;
    calibrated_=calibrated>.5;
  } else if(external_estop_.load()) return Fault::Estop;
  for(std::size_t i=0;i<12;++i) {
    sensor.position_sdk[i]=state_interfaces_[state_indices_[i][0]].get_value();
    sensor.velocity_sdk[i]=state_interfaces_[state_indices_[i][1]].get_value();
    const double effort=state_interfaces_[state_indices_[i][2]].get_value();
    if(!std::isfinite(sensor.position_sdk[i]) || !std::isfinite(sensor.velocity_sdk[i]) || !std::isfinite(effort)) return Fault::Feedback;
    if(hybrid()) {
      const double temperature=state_interfaces_[state_indices_[i][3]].get_value();
      const double valid=state_interfaces_[state_indices_[i][4]].get_value();
      if(!std::isfinite(valid) || valid<.5 || !std::isfinite(temperature)) return Fault::Feedback;
      if(temperature>max_temperature_) return Fault::Temperature;
    }
    // Encoder positions have no URDF zero until the explicit calibration operation.
    if(calibrated_ && (sensor.position_sdk[i]<lower_[i]-joint_tolerance_ || sensor.position_sdk[i]>upper_[i]+joint_tolerance_)) return Fault::Limits;
    if(std::abs(effort)>effort_limit_[i]*1.15) return Fault::Effort;
    if(std::abs(sensor.velocity_sdk[i])>velocity_limit_[i]*1.2) return Fault::Velocity;
  }
  if(need_imu) {
    if(!imu.valid || now-imu.received>imu_timeout_ || now<imu.received) return Fault::Imu;
    sensor.orientation_wxyz=imu.orientation; sensor.angular_velocity_body=imu.gyro;
    const double upright=1-2*(imu.orientation[1]*imu.orientation[1]+imu.orientation[2]*imu.orientation[2]);
    if(upright<std::cos(max_tilt_)) return Fault::Tilt;
  }
  return Fault::None;
}
void RlController::transition(Mode next,double now) {
  ++epoch_; submitted_=0; accepted_=0; previous_action_.fill(0);
  entered_=now; next_inference_=now; settled_since_=0; mode_=next;
  if(next==Mode::Calibrating) calibration_zero_seen_=false;
  published_mode_.store(static_cast<int>(mode_)); published_fault_.store(static_cast<int>(fault_));
}
void RlController::latch(Fault fault,double now) {
  if(mode_!=Mode::Fault) {fault_=fault; transition(Mode::Fault,now);}
}
void RlController::writePassive() {
  for(std::size_t i=0;i<12;++i) {
    const double measured=state_interfaces_[state_indices_[i][0]].get_value();
    if(hybrid()) {
      const std::array<double,5> values{std::isfinite(measured)?measured:0,0,0,0,enable_actuation_ && mode_!=Mode::Shadow?safe_damping_:0};
      for(std::size_t j=0;j<5;++j) command_interfaces_[command_indices_[i][j]].set_value(values[j]);
    } else {
      const double dq=state_interfaces_[state_indices_[i][1]].get_value();
      const double effort=enable_actuation_ && mode_!=Mode::Shadow && std::isfinite(dq)?std::clamp(-safe_damping_*dq,-effort_limit_[i],effort_limit_[i]):0;
      command_interfaces_[command_indices_[i][2]].set_value(effort);
    }
  }
  if(hybrid()) {
    command_interfaces_[system_commands_[0]].set_value(0);
    // An asserted hardware emergency_stop overrides gains with driver damping.
    // Keep it clear when actuation is disabled so zero gains remain zero.
    // The physical cutoff remains independent of this software damping command.
    command_interfaces_[system_commands_[1]].set_value(mode_==Mode::Fault && enable_actuation_?1:0);
  }
}
void RlController::writeTarget(const JointArray& requested,const SensorFrame& sensor,double dt) {
  const bool standing=mode_==Mode::Standing || mode_==Mode::Ready;
  const double proportional=standing?stand_kp_:kp_;
  const double derivative=standing?stand_kd_:kd_;
  for(std::size_t i=0;i<12;++i) {
    double q=std::clamp(requested[i],lower_[i]+target_margin_,upper_[i]-target_margin_);
    if(target_rate_>0) q=std::clamp(q,target_[i]-target_rate_*dt,target_[i]+target_rate_*dt);
    q=std::clamp(q,lower_[i]+target_margin_,upper_[i]-target_margin_);
    target_[i]=q;
    if(hybrid()) {
      // Keep predicted joint-side PD torque inside the actuator bound. Reducing Kp
      // avoids rewriting the raw action retained in the policy observation history.
      const double damping=-derivative*sensor.velocity_sdk[i];
      const double effective_kd=std::abs(damping)>effort_limit_[i] ? effort_limit_[i]/std::max(std::abs(sensor.velocity_sdk[i]),1e-9) : derivative;
      const double headroom=std::max(0.,effort_limit_[i]-std::abs(effective_kd*sensor.velocity_sdk[i]));
      const double effective_kp=std::min(proportional,headroom/std::max(std::abs(q-sensor.position_sdk[i]),1e-9));
      const std::array<double,5> values{q,0,0,effective_kp,effective_kd};
      for(std::size_t j=0;j<5;++j) command_interfaces_[command_indices_[i][j]].set_value(values[j]);
    } else {
      command_interfaces_[command_indices_[i][2]].set_value(std::clamp(proportional*(q-sensor.position_sdk[i])-derivative*sensor.velocity_sdk[i],-effort_limit_[i],effort_limit_[i]));
    }
  }
}
controller_interface::return_type RlController::update(const rclcpp::Time&,const rclcpp::Duration& period) {
  const double now=nowSteady(), dt=period.seconds();
  const auto imu=*imu_buffer_.readFromRT();
  SensorFrame sensor;
  auto health=readSensors(sensor,imu,now,mode_!=Mode::Passive && mode_!=Mode::Calibrating);
  const Request request=request_.exchange(Request::None);
  if(request==Request::Estop || external_estop_.load()) latch(Fault::Estop,now);
  else if(request==Request::Passive) {
    // A mode command alone cannot clear a still-present sensor or actuator fault.
    const auto reset_health=readSensors(sensor,imu,now,mode_==Mode::Fault);
    if(reset_health==Fault::None) {fault_=Fault::None;transition(Mode::Passive,now);}
  }
  if(mode_!=Mode::Passive && mode_!=Mode::Fault && health!=Fault::None) latch(health,now);
  if((mode_==Mode::Standing || mode_==Mode::Ready || mode_==Mode::Rl || mode_==Mode::Shadow) && !calibrated_) latch(Fault::Feedback,now);
  if(mode_!=Mode::Passive && mode_!=Mode::Fault && (!std::isfinite(dt) || dt<=0 || dt>max_control_period_)) latch(Fault::Period,now);
  writePassive();
  if(mode_==Mode::Fault) return controller_interface::return_type::OK;
  if(mode_==Mode::Passive) {
    if(health!=Fault::None) return controller_interface::return_type::OK;
    if(request==Request::Calibrate && hybrid()) transition(Mode::Calibrating,now);
    else if(request==Request::Stand && enable_actuation_ && calibrated_ && readSensors(sensor,imu,now,true)==Fault::None) {
      stand_start_=sensor.position_sdk; target_=stand_start_; transition(Mode::Standing,now);
    } else if(request==Request::Shadow && calibrated_ && readSensors(sensor,imu,now,true)==Fault::None) {
      transition(Mode::Shadow,now);
    }
  }
  if(mode_==Mode::Calibrating) {
    // Recalibration must observe the hardware clear its old zero before accepting
    // a new calibrated flag. A persistent old flag cannot complete this handshake.
    if(!calibrated_) calibration_zero_seen_=true;
    command_interfaces_[system_commands_[0]].set_value(calibration_zero_seen_?1:-1);
    if(calibration_zero_seen_ && calibrated_) transition(Mode::Passive,now);
    else if(now-entered_>calibration_timeout_) latch(Fault::CalibrationTimeout,now);
  } else if(mode_==Mode::Standing) {
    const double u=std::clamp((now-entered_)/stand_duration_,0.,1.);
    const double blend=u*u*(3-2*u);
    JointArray desired{};
    bool settled=u>=1;
    for(std::size_t i=0;i<12;++i) {
      desired[i]=stand_start_[i]+blend*(PolicyCore::defaultPositionsSdk()[i]-stand_start_[i]);
      settled=settled && std::abs(sensor.position_sdk[i]-PolicyCore::defaultPositionsSdk()[i])<=stand_tolerance_ && std::abs(sensor.velocity_sdk[i])<=stand_velocity_;
    }
    writeTarget(desired,sensor,dt);
    if(settled) {if(settled_since_==0) settled_since_=now; if(now-settled_since_>=.3) transition(Mode::Ready,now);}
    else settled_since_=0;
    if(mode_==Mode::Standing && now-entered_>stand_timeout_) latch(Fault::StandTimeout,now);
  } else if(mode_==Mode::Ready) {
    writeTarget(PolicyCore::defaultPositionsSdk(),sensor,dt);
    if(request==Request::Rl && enable_actuation_) transition(Mode::Rl,now);
  }
  if(mode_==Mode::Rl || mode_==Mode::Shadow) {
    const auto result=*result_buffer_.readFromRT();
    if(result.epoch==epoch_ && result.sequence>accepted_) {
      if(!result.valid) latch(Fault::Policy,now);
      else if(result.duration>deadline_ || now-result.observed>policy_timeout_) latch(Fault::Deadline,now);
      else {accepted_=result.sequence;previous_action_=result.action;}
    }
    const double age=accepted_>0 && result.epoch==epoch_?now-result.observed:now-entered_;
    if(age>policy_timeout_) latch(Fault::Deadline,now);
    if(mode_==Mode::Rl || mode_==Mode::Shadow) {
      if(now-next_inference_>=kPolicyPeriodS) latch(Fault::Deadline,now);
      if((mode_==Mode::Rl || mode_==Mode::Shadow) && now>=next_inference_) {
        std::unique_lock<std::mutex> lock(work_mutex_,std::try_to_lock);
        if(lock.owns_lock() && !has_work_) {
          const auto command=*command_buffer_.readFromRT();
          pending_.sensor=sensor;
          pending_.command=now-command.received<=command_timeout_?command.velocity:std::array<double,3>{};
          pending_.previous_action=previous_action_;
          pending_.epoch=epoch_;pending_.sequence=++submitted_;pending_.observed=now;
          has_work_=true;next_inference_+=kPolicyPeriodS;
          lock.unlock();work_ready_.notify_one();
        }
      }
      if(mode_==Mode::Rl) writeTarget(accepted_?PolicyCore{}.decode(previous_action_):PolicyCore::defaultPositionsSdk(),sensor,dt);
      // Shadow computes policy outputs but deliberately retains passive motor commands.
    }
  }
  // Apply the zero-gain shadow command on the very transition cycle as well.
  if(mode_==Mode::Fault || mode_==Mode::Shadow) writePassive();
  return controller_interface::return_type::OK;
}
}  // namespace custom_dog_rl
PLUGINLIB_EXPORT_CLASS(custom_dog_rl::RlController,controller_interface::ControllerInterface)
