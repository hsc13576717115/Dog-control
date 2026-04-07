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
      _calfTotalGearRatio(static_cast<float>(drive_parameters.calf_total_gear_ratio)) {
    _calibOffset.fill(0.0f);
    _activeLegs = {0, 1, 2, 3};

    std::cout << "[IOSDK] 初始化完成" << std::endl;
    std::cout << "[IOSDK] 电机串口配置"
              << " FR=" << _serialPorts[0]
              << " FL=" << _serialPorts[1]
              << " RR=" << _serialPorts[2]
              << " RL=" << _serialPorts[3]
              << std::endl;

    openSerialPorts();
    initializeMotorMetadata();

    _lastCalibPromptTimeSec = NowSec();
    std::cout << "[IOSDK] 手动调整至趴下姿态后，按 START 触发校准" << std::endl;
}

IOSDK::~IOSDK() {
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
                      << " baud=4000000" << std::endl;
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

float IOSDK::gearRatioForJoint(int joint) const {
    return (joint == 2) ? _calfTotalGearRatio : _baseGearRatio;
}

float IOSDK::calibrationTargetUserAngle(int leg, int joint) const {
    const bool left_leg = (leg == 1 || leg == 3);
    const float sign = left_leg ? -1.0f : 1.0f;

    if (joint == 0) {
        return _calibrationHipAngleRad;
    }
    if (joint == 1) {
        return sign * _calibrationThighAngleRad;
    }
    if (joint == 2) {
        return sign * _calibrationCalfAngleRad;
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
    const float q_cmd = _isCalibrated ? (user_cmd.q + _calibOffset[motor_id]) : user_cmd.q;

    auto& motor_cmd = _motorCmd[motor_id];
    motor_cmd.id = joint;
    motor_cmd.q = q_cmd * gear;
    motor_cmd.dq = user_cmd.dq * gear;
    motor_cmd.kp = user_cmd.Kp;
    motor_cmd.kd = user_cmd.Kd;
    motor_cmd.tau = user_cmd.tau;
    motor_cmd.mode = user_cmd.mode;
}

void IOSDK::updateMotorStateFromFeedback(int leg, int joint, LowlevelState* state) {
    const int motor_id = leg * 3 + joint;
    const float gear = gearRatioForJoint(joint);
    const float q_feedback = _motorData[motor_id].q / gear;

    auto& motor_state = state->motorState[motor_id];
    motor_state.q = _isCalibrated ? (q_feedback - _calibOffset[motor_id]) : q_feedback;
    motor_state.dq = _motorData[motor_id].dq / gear;
    motor_state.tauEst = _motorData[motor_id].tau;
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

    for (int leg : _activeLegs) {
        sendReceiveLeg(leg, cmd, state);
    }
}

#endif  // COMPILE_WITH_REAL_ROBOT
