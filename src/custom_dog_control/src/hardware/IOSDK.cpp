#ifdef COMPILE_WITH_REAL_ROBOT

#include "custom_dog_control/hardware/IOSDK.hpp"

#include <cmath>
#include <iostream>

namespace {

constexpr const char* LEG_NAMES[4] = {"FR", "FL", "RR", "RL"};
constexpr const char* JOINT_NAMES[3] = {"hip", "thigh", "calf"};

}  // namespace

namespace custom_dog_control {

IOSDK::IOSDK(const DriveParameters& drive_parameters)
    : serial_ports_(drive_parameters.serial_ports),
      base_gear_ratio_(static_cast<float>(drive_parameters.base_gear_ratio)),
      calf_total_gear_ratio_(static_cast<float>(drive_parameters.calf_total_gear_ratio)),
      use_parallel_leg_io_(drive_parameters.use_parallel_leg_io),
      calibration_pose_(drive_parameters.calibration_pose) {
    calibration_offsets_.fill(0.0f);
    for (std::size_t i = 0; i < motor_directions_.size(); ++i) {
        motor_directions_[i] = static_cast<float>(drive_parameters.motor_directions[i]);
    }
    active_legs_ = {0, 1, 2, 3};

    std::cout << "[IOSDK] 初始化完成" << std::endl;
    std::cout << "[IOSDK] 电机串口配置"
              << " FR=" << serial_ports_[0]
              << " FL=" << serial_ports_[1]
              << " RR=" << serial_ports_[2]
              << " RL=" << serial_ports_[3]
              << std::endl;
    std::cout << "[IOSDK] 腿部通信模式="
              << (use_parallel_leg_io_ ? "parallel_4workers" : "single_thread")
              << std::endl;

    OpenSerialPorts();
    InitializeMotorMetadata();

}

IOSDK::~IOSDK() {
    Deactivate();
    for (SerialPort* serial : serials_) {
        delete serial;
    }
}

void IOSDK::Activate() {
    if (active_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        workers_stopping_ = false;
        dispatch_epoch_ = 0;
        completed_workers_ = 0;
        active_command_ = nullptr;
        active_state_ = nullptr;
    }
    StartWorkers();
    active_ = true;
}

void IOSDK::Deactivate() {
    if (!active_) {
        return;
    }
    StopWorkers();
    const auto zero_cmd = BuildZeroTorqueCommand();
    for (int leg : active_legs_) {
        SendDirectLegCommand(leg, zero_cmd);
    }
    active_ = false;
}

void IOSDK::OpenSerialPorts() {
    for (int leg = 0; leg < 4; ++leg) {
        try {
            serials_.push_back(new SerialPort(
                serial_ports_[leg], 16, 4000000, 2000, BlockYN::YES,
                bytesize_t::eightbits, parity_t::parity_none,
                stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none));
            std::cout << "[IOSDK] 电机串口已打开"
                      << " leg=" << LEG_NAMES[leg]
                      << " port=" << serial_ports_[leg]
                      << " baud=4000000"
                      << " timeout_us=10000" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[IOSDK][ERROR] 电机串口打开失败"
                      << " leg=" << LEG_NAMES[leg]
                      << " port=" << serial_ports_[leg]
                      << " reason=" << e.what()
                      << std::endl;
            throw;
        }
    }
}

void IOSDK::InitializeMotorMetadata() {
    for (int i = 0; i < 12; ++i) {
        motor_commands_[i].motorType = MotorType::GO_M8010_6;
        motor_commands_[i].mode = static_cast<unsigned short>(MotorMode::FOC);
        motor_commands_[i].hex_len = 23;

        motor_data_[i].motorType = MotorType::GO_M8010_6;
        motor_data_[i].hex_len = 31;
    }
}

std::array<ActuatorCommand, 3> IOSDK::BuildZeroTorqueCommand() const {
    std::array<ActuatorCommand, 3> cmds;
    for (auto& cmd : cmds) {
        cmd.mode = static_cast<std::uint32_t>(ActuatorControlMode::COMPOUND);
    }
    return cmds;
}

void IOSDK::SendDirectLegCommand(int leg, const std::array<ActuatorCommand, 3>& user_cmds) {
    SerialPort* serial = serials_[leg];
    if (serial == nullptr) {
        return;
    }

    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        PopulateMotorCommand(leg, joint, user_cmds[joint]);
        if (!serial->sendRecv(&motor_commands_[motor_id], &motor_data_[motor_id])) {
            std::cerr << "[IOSDK][WARN] 预对位阶段电机无回包"
                      << " leg=" << LEG_NAMES[leg]
                      << " joint=" << JOINT_NAMES[joint]
                      << " port=" << serial_ports_[leg]
                      << std::endl;
        }
    }
}

void IOSDK::StartWorkers() {
    if (!use_parallel_leg_io_) {
        return;
    }

    try {
        for (int leg = 0; leg < 4; ++leg) {
            workers_[leg] = std::thread(&IOSDK::WorkerLoop, this, leg);
        }
    } catch (...) {
        StopWorkers();
        throw;
    }
}

void IOSDK::StopWorkers() {
    if (!use_parallel_leg_io_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        workers_stopping_ = true;
    }
    dispatch_cv_.notify_all();
    completed_cv_.notify_all();

    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void IOSDK::WorkerLoop(int leg) {
    std::size_t local_epoch = 0;

    while (true) {
        const LowLevelCommand* cmd = nullptr;
        LowLevelState* state = nullptr;

        {
            std::unique_lock<std::mutex> lock(worker_mutex_);
            dispatch_cv_.wait(lock, [this, &local_epoch] {
                return workers_stopping_ || dispatch_epoch_ != local_epoch;
            });

            if (workers_stopping_) {
                return;
            }

            local_epoch = dispatch_epoch_;
            cmd = active_command_;
            state = active_state_;
        }

        const bool leg_active = (active_legs_.find(leg) != active_legs_.end());
        if (leg_active) {
            try {
                SendReceiveLeg(leg, *cmd, *state);
            } catch (...) {
                for (int joint = 0; joint < 3; ++joint) {
                    MarkMotorOffline(leg, joint, *state);
                }
            }
        }

        if (leg_active) {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            ++completed_workers_;
            if (completed_workers_ >= active_legs_.size()) {
                completed_cv_.notify_one();
            }
        }
    }
}

float IOSDK::GearRatioForJoint(int joint) const {
    return (joint == 2) ? calf_total_gear_ratio_ : base_gear_ratio_;
}

void IOSDK::CalibrateLeg(int leg) {
    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        const float gear = GearRatioForJoint(joint);
        const float dir = motor_directions_[motor_id];
        const double q_urdf_uncalibrated = MotorPositionToUncalibratedUrdf(
            motor_data_[motor_id].q, gear, dir);
        calibration_offsets_[motor_id] = static_cast<float>(
            CalibrationOffset(q_urdf_uncalibrated, joint, calibration_pose_));
    }
}

void IOSDK::TryCalibrate(bool calibration_requested) {
    if (calibrated_ || !calibration_requested) {
        return;
    }

    std::cout << "[IOSDK] 检测到 START，开始校准" << std::endl;
    for (int leg : active_legs_) {
        CalibrateLeg(leg);
    }
    calibrated_ = true;
    std::cout << "[IOSDK] 校准完成" << std::endl;
}

void IOSDK::FinalizeCycleCalibration(
    bool calibration_requested, LowLevelState& state) {
    const int failures = current_cycle_failures_.load(std::memory_order_relaxed);
    last_cycle_failures_.store(failures, std::memory_order_release);
    const bool was_calibrated = calibrated_;
    if (failures == 0) {
        TryCalibrate(calibration_requested);
    }
    if (!was_calibrated && calibrated_) {
        for (int leg : active_legs_) {
            for (int joint = 0; joint < static_cast<int>(kJointsPerLeg); ++joint) {
                UpdateMotorStateFromFeedback(leg, joint, state);
            }
        }
    }
}

void IOSDK::PopulateMotorCommand(int leg, int joint, const ActuatorCommand& user_cmd) {
    const int motor_id = leg * 3 + joint;
    const float gear = GearRatioForJoint(joint);
    const float gear_sq = gear * gear;
    const float dir = motor_directions_[leg * 3 + joint];
    auto& motor_cmd = motor_commands_[motor_id];
    motor_cmd.id = joint;
    motor_cmd.q = static_cast<float>(UrdfPositionToMotor(
        user_cmd.position,
        calibrated_ ? calibration_offsets_[motor_id] : 0.0F,
        gear, dir));
    motor_cmd.dq = user_cmd.velocity * gear * dir;
    motor_cmd.kp = user_cmd.kp / gear_sq;
    motor_cmd.kd = user_cmd.kd / gear_sq;
    motor_cmd.tau = (user_cmd.effort / gear) * dir;
    motor_cmd.mode = user_cmd.mode;
}

void IOSDK::UpdateMotorStateFromFeedback(int leg, int joint, LowLevelState& state) {
    const int motor_id = leg * 3 + joint;
    const float gear = GearRatioForJoint(joint);
    const float dir = motor_directions_[leg * 3 + joint];
    const float q_feedback = static_cast<float>(MotorPositionToUncalibratedUrdf(
        motor_data_[motor_id].q, gear, dir));

    auto& motor_state = state.motors[motor_id];
    motor_state.position = calibrated_
                               ? static_cast<float>(CalibratedUrdfPosition(
                                     q_feedback, calibration_offsets_[motor_id]))
                               : q_feedback;
    motor_state.velocity = (motor_data_[motor_id].dq / gear) * dir;
    motor_state.effort = (motor_data_[motor_id].tau * gear) * dir;
    motor_state.temperature = motor_data_[motor_id].temp;
    motor_state.fault = motor_data_[motor_id].merror;
}

void IOSDK::MarkMotorOffline(int leg, int joint, LowLevelState& state) const {
    const int motor_id = leg * 3 + joint;
    auto& motor_state = state.motors[motor_id];
    motor_state = {};
    motor_state.fault = 0xFF;
    current_cycle_failures_.fetch_add(1, std::memory_order_relaxed);
    total_failures_.fetch_add(1, std::memory_order_relaxed);
}

void IOSDK::SendReceiveLeg(
    int leg, const LowLevelCommand& command, LowLevelState& state) {
    SerialPort* serial = serials_[leg];
    if (serial == nullptr) {
        return;
    }

    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        PopulateMotorCommand(leg, joint, command.motors[motor_id]);
        if (serial->sendRecv(&motor_commands_[motor_id], &motor_data_[motor_id])) {
            UpdateMotorStateFromFeedback(leg, joint, state);
        } else {
            MarkMotorOffline(leg, joint, state);
        }
    }
}

void IOSDK::SendReceive(
    const LowLevelCommand& command, LowLevelState& state,
    bool calibration_requested) {
    if (!active_ || serials_.size() != kLegCount) {
        return;
    }

    current_cycle_failures_.store(0, std::memory_order_relaxed);

    if (!use_parallel_leg_io_) {
        for (int leg : active_legs_) {
            SendReceiveLeg(leg, command, state);
        }
        FinalizeCycleCalibration(calibration_requested, state);
        return;
    }

    const std::size_t active_workers = active_legs_.size();
    if (active_workers == 0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        active_command_ = &command;
        active_state_ = &state;
        completed_workers_ = 0;
        ++dispatch_epoch_;
    }
    dispatch_cv_.notify_all();

    std::unique_lock<std::mutex> lock(worker_mutex_);
    completed_cv_.wait(lock, [this, active_workers] {
        return workers_stopping_ || completed_workers_ >= active_workers;
    });
    FinalizeCycleCalibration(calibration_requested, state);
}

}  // namespace custom_dog_control

#endif  // COMPILE_WITH_REAL_ROBOT
