#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"

#include <chrono>
#include <cmath>
#include <iostream>

namespace {

constexpr const char* LEG_NAMES[4] = {"FR", "FL", "RR", "RL"};
constexpr const char* JOINT_NAMES[3] = {"hip", "thigh", "calf"};


double NowSec() {
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    return duration<double>(steady_clock::now() - start).count();
}

}  // namespace

IOSDK::IOSDK(const qr_guide::DriveParameters& drive_parameters)
    : _serialPorts(drive_parameters.serial_ports),
      _baseGearRatio(static_cast<float>(drive_parameters.base_gear_ratio)),
      _calfTotalGearRatio(static_cast<float>(drive_parameters.calf_total_gear_ratio)),
      _useParallelLegIo(drive_parameters.use_parallel_leg_io) {
    _calibOffset.fill(0.0f);
    _activeLegs = {0, 1, 2, 3};

    std::cout << "[IOSDK] 初始化完成" << std::endl;
    std::cout << "[IOSDK] 电机串口配置"
              << " FR=" << _serialPorts[0]
              << " FL=" << _serialPorts[1]
              << " RR=" << _serialPorts[2]
              << " RL=" << _serialPorts[3]
              << std::endl;
    std::cout << "[IOSDK] 腿部通信模式="
              << (_useParallelLegIo ? "parallel_4workers" : "single_thread")
              << std::endl;

    openSerialPorts();
    initializeMotorMetadata();
    runStartupPoseAlignment();
    startWorkers();

    _lastCalibPromptTimeSec = NowSec();
    std::cout << "[IOSDK] 预对位完成，请确认趴下姿态后按 START 触发校准" << std::endl;
}

IOSDK::~IOSDK() {
    stopWorkers();
    for (SerialPort* serial : _serials) {
        delete serial;
    }
}

void IOSDK::openSerialPorts() {
    for (int leg = 0; leg < 4; ++leg) {
        try {
            _serials.push_back(new SerialPort(
                _serialPorts[leg], 16, 4000000, 2000, BlockYN::YES,
                bytesize_t::eightbits, parity_t::parity_none,
                stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none));
            std::cout << "[IOSDK] 电机串口已打开"
                      << " leg=" << LEG_NAMES[leg]
                      << " port=" << _serialPorts[leg]
                      << " baud=4000000"
                      << " timeout_us=10000" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[IOSDK][ERROR] 电机串口打开失败"
                      << " leg=" << LEG_NAMES[leg]
                      << " port=" << _serialPorts[leg]
                      << " reason=" << e.what()
                      << std::endl;
            throw;
        }
    }
}

void IOSDK::initializeMotorMetadata() {
    for (int i = 0; i < 12; ++i) {
        _motorCmd[i].motorType = MotorType::GO_M8010_6;
        _motorCmd[i].mode = static_cast<unsigned short>(MotorMode::FOC);
        _motorCmd[i].hex_len = 23;

        _motorData[i].motorType = MotorType::GO_M8010_6;
        _motorData[i].hex_len = 31;
    }
}

std::array<UserLowlevel::MotorCmd, 3> IOSDK::buildAlignmentCommand(int leg, bool enable_calf) const {
    std::array<UserLowlevel::MotorCmd, 3> cmds;
    const bool left_leg = (leg == 1 || leg == 3);
    const bool front_leg = (leg == 0 || leg == 1);
    // 用户关节定义下：右腿向内展/下压/上抬取负，左腿取正。
    const float side_sign = left_leg ? 1.0f : -1.0f;
    const float hip_sign = front_leg ? -side_sign : side_sign;

    for (auto& cmd : cmds) {
        cmd.mode = static_cast<unsigned int>(ControlMode::COMPOUND);
        cmd.q = 0.0f;
        cmd.dq = 0.0f;
        cmd.Kp = 0.0f;
        cmd.Kd = 0.0f;
        cmd.tau = 0.0f;
    }

    cmds[0].tau = hip_sign * _startupHipTauNm;
    cmds[1].tau = -side_sign * _startupThighTauNm;
    cmds[2].tau = enable_calf ? (side_sign * _startupCalfTauNm) : 0.0f;
    return cmds;
}

std::array<UserLowlevel::MotorCmd, 3> IOSDK::buildZeroTorqueCommand() const {
    std::array<UserLowlevel::MotorCmd, 3> cmds;
    for (auto& cmd : cmds) {
        cmd.mode = static_cast<unsigned int>(ControlMode::COMPOUND);
        cmd.q = 0.0f;
        cmd.dq = 0.0f;
        cmd.Kp = 0.0f;
        cmd.Kd = 0.0f;
        cmd.tau = 0.0f;
    }
    return cmds;
}

void IOSDK::sendDirectLegCommand(int leg, const std::array<UserLowlevel::MotorCmd, 3>& user_cmds) {
    SerialPort* serial = _serials[leg];
    if (serial == nullptr) {
        return;
    }

    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        populateMotorCommand(leg, joint, user_cmds[joint]);
        if (!serial->sendRecv(&_motorCmd[motor_id], &_motorData[motor_id])) {
            std::cerr << "[IOSDK][WARN] 预对位阶段电机无回包"
                      << " leg=" << LEG_NAMES[leg]
                      << " joint=" << JOINT_NAMES[joint]
                      << " port=" << _serialPorts[leg]
                      << std::endl;
        }
    }
}

void IOSDK::refreshMotorFeedback() {
    const auto zero_cmd = buildZeroTorqueCommand();
    for (int leg : _activeLegs) {
        sendDirectLegCommand(leg, zero_cmd);
    }
}

void IOSDK::runStartupPoseAlignment() {
    if (_startupAlignmentDone) {
        return;
    }

    std::cout << "[IOSDK] 启动预对位: 髋内展 / 大腿先下压 / 小腿后上抬"
              << " joint_tau=[" << _startupHipTauNm
              << ", " << _startupThighTauNm
              << ", " << _startupCalfTauNm << "]Nm"
              << std::endl;
    refreshMotorFeedback();

    for (int leg : _activeLegs) {
        sendDirectLegCommand(leg, buildAlignmentCommand(leg, false));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(_startupAlignmentPhase1Ms));

    for (int leg : _activeLegs) {
        sendDirectLegCommand(leg, buildAlignmentCommand(leg, true));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(_startupAlignmentPhase2Ms));

    const auto zero_cmd = buildZeroTorqueCommand();
    for (int leg : _activeLegs) {
        sendDirectLegCommand(leg, zero_cmd);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(_startupAlignmentReleaseMs));

    refreshMotorFeedback();
    _startupAlignmentDone = true;
    std::cout << "[IOSDK] 启动预对位完成" << std::endl;
}

void IOSDK::startWorkers() {
    if (!_useParallelLegIo) {
        return;
    }

    try {
        for (int leg = 0; leg < 4; ++leg) {
            _workers[leg] = std::thread(&IOSDK::workerLoop, this, leg);
        }
    } catch (...) {
        stopWorkers();
        throw;
    }
}

void IOSDK::stopWorkers() {
    if (!_useParallelLegIo) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(_workerMutex);
        _workersStopping = true;
    }
    _dispatchCv.notify_all();
    _completedCv.notify_all();

    for (auto& worker : _workers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void IOSDK::workerLoop(int leg) {
    std::size_t local_epoch = 0;

    while (true) {
        const UserLowlevel::LowlevelCmd* cmd = nullptr;
        LowlevelState* state = nullptr;

        {
            std::unique_lock<std::mutex> lock(_workerMutex);
            _dispatchCv.wait(lock, [this, &local_epoch] {
                return _workersStopping || _dispatchEpoch != local_epoch;
            });

            if (_workersStopping) {
                return;
            }

            local_epoch = _dispatchEpoch;
            cmd = _activeCmd;
            state = _activeState;
        }

        const bool leg_active = (_activeLegs.find(leg) != _activeLegs.end());
        if (leg_active) {
            sendReceiveLeg(leg, cmd, state);
        }

        if (leg_active) {
            std::lock_guard<std::mutex> lock(_workerMutex);
            ++_completedWorkers;
            if (_completedWorkers >= _activeLegs.size()) {
                _completedCv.notify_one();
            }
        }
    }
}

float IOSDK::gearRatioForJoint(int joint) const {
    return (joint == 2) ? _calfTotalGearRatio : _baseGearRatio;
}

float IOSDK::calibrationTargetUserAngle(int leg, int joint) const {
    const bool left_leg = (leg == 1 || leg == 3);
    const float sign = left_leg ? -1.0f : 1.0f;
    const float dir = MOTOR_DIRECTION[leg][joint];

    if (joint == 0) {
        return _calibrationHipAngleRad * dir;
    }
    if (joint == 1) {
        return sign * _calibrationThighAngleRad * dir;
    }
    if (joint == 2) {
        return sign * _calibrationCalfAngleRad * dir;
    }
    return 0.0f;
}

void IOSDK::maybePrintCalibrationReminder() {
    if (_isCalibrated) {
        return;
    }

    const double now = NowSec();
    if (now - _lastCalibPromptTimeSec >= _calibPromptIntervalSec) {
        std::cout << "[IOSDK] 等待校准，按 START 完成 L 型零位采集" << std::endl;
        _lastCalibPromptTimeSec = now;
    }
}

void IOSDK::calibrateLeg(int leg) {
    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        const float gear = gearRatioForJoint(joint);
        const float q = _motorData[motor_id].q / gear;
        _calibOffset[motor_id] = q - calibrationTargetUserAngle(leg, joint);
    }
}

void IOSDK::tryCalibrate(const LowlevelState& state) {
    if (_isCalibrated || state.userCmd != _calibTriggerKey) {
        return;
    }

    std::cout << "[IOSDK] 检测到 START，开始校准" << std::endl;
    for (int leg : _activeLegs) {
        calibrateLeg(leg);
    }
    _isCalibrated = true;
    std::cout << "[IOSDK] 校准完成" << std::endl;
}

void IOSDK::populateMotorCommand(int leg, int joint, const UserLowlevel::MotorCmd& user_cmd) {
    const int motor_id = leg * 3 + joint;
    const float gear = gearRatioForJoint(joint);
    const float gear_sq = gear * gear;
    const float dir = MOTOR_DIRECTION[leg][joint];
    const float q_cmd = _isCalibrated ? (user_cmd.q + _calibOffset[motor_id]) : user_cmd.q;

    auto& motor_cmd = _motorCmd[motor_id];
    motor_cmd.id = joint;
    motor_cmd.q = q_cmd * gear * dir;
    motor_cmd.dq = user_cmd.dq * gear * dir;
    motor_cmd.kp = user_cmd.Kp / gear_sq;
    motor_cmd.kd = user_cmd.Kd / gear_sq;
    motor_cmd.tau = (user_cmd.tau / gear) * dir;
    motor_cmd.mode = user_cmd.mode;
}

void IOSDK::updateMotorStateFromFeedback(int leg, int joint, LowlevelState* state) {
    const int motor_id = leg * 3 + joint;
    const float gear = gearRatioForJoint(joint);
    const float dir = MOTOR_DIRECTION[leg][joint];
    const float q_feedback = (_motorData[motor_id].q / gear) * dir;

    auto& motor_state = state->motorState[motor_id];
    motor_state.q = _isCalibrated ? (q_feedback - _calibOffset[motor_id]) : q_feedback;
    motor_state.dq = (_motorData[motor_id].dq / gear) * dir;
    motor_state.tauEst = (_motorData[motor_id].tau * gear) * dir;
    motor_state.temp = _motorData[motor_id].temp;
    motor_state.fault = _motorData[motor_id].merror;
}

void IOSDK::markMotorOffline(int leg, int joint, LowlevelState* state) const {
    const int motor_id = leg * 3 + joint;
    auto& motor_state = state->motorState[motor_id];
    motor_state = {};
    motor_state.fault = 0xFF;

    std::cerr << "[IOSDK][WARN] 电机无回包"
              << " leg=" << LEG_NAMES[leg]
              << " joint=" << JOINT_NAMES[joint]
              << " port=" << _serialPorts[leg]
              << " motor_id=" << joint
              << " state_index=" << motor_id
              << std::endl;
}

void IOSDK::sendReceiveLeg(int leg, const UserLowlevel::LowlevelCmd* cmd, LowlevelState* state) {
    SerialPort* serial = _serials[leg];
    if (serial == nullptr) {
        return;
    }

    for (int joint = 0; joint < 3; ++joint) {
        const int motor_id = leg * 3 + joint;
        populateMotorCommand(leg, joint, cmd->motorCmd[motor_id]);
        if (serial->sendRecv(&_motorCmd[motor_id], &_motorData[motor_id])) {
            updateMotorStateFromFeedback(leg, joint, state);
        } else {
            markMotorOffline(leg, joint, state);
        }
    }
}

void IOSDK::sendRecv(const UserLowlevel::LowlevelCmd* cmd, LowlevelState* state) {
    if (cmd == nullptr || state == nullptr || _serials.size() != 4) {
        return;
    }

    maybePrintCalibrationReminder();
    tryCalibrate(*state);

    if (!_useParallelLegIo) {
        for (int leg : _activeLegs) {
            sendReceiveLeg(leg, cmd, state);
        }
        return;
    }

    const std::size_t active_workers = _activeLegs.size();
    if (active_workers == 0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(_workerMutex);
        _activeCmd = cmd;
        _activeState = state;
        _completedWorkers = 0;
        ++_dispatchEpoch;
    }
    _dispatchCv.notify_all();

    std::unique_lock<std::mutex> lock(_workerMutex);
    _completedCv.wait(lock, [this, active_workers] {
        return _workersStopping || _completedWorkers >= active_workers;
    });
}

#endif  // COMPILE_WITH_REAL_ROBOT
