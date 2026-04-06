#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/IOSDK.h"
#include "message/LowlevelState.h"
#include "message/LowlevelCmd.h"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "FSM/State_SwingTest.h"
#include "common/enumClass.h"
#include "interface/CmdPanel.h"
#include "common/mathTools.h"

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#ifdef COMPILE_WITH_MOVE_BASE
#include <sensor_msgs/msg/joint_state.hpp>
#endif

/* ===================== 常量定义 ===================== */

static constexpr float BASE_GEAR_RATIO = 6.33f;
static constexpr float CALF_EXTRA_GEAR = 2.0f;
static constexpr float CALF_TOTAL_GEAR = BASE_GEAR_RATIO * CALF_EXTRA_GEAR;

const std::vector<std::string> SERIAL_PORTS = {
    "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS7", "/dev/ttyS8"
};

static const Vec3 pHip2B(0.1525f, -0.0565f, 0.0f);

/* ===================== 手柄映射 ===================== */

enum Xbox360JoyMap {
    BTN_A = 0, BTN_B = 1, BTN_X = 2, BTN_Y = 3,
    BTN_LB = 4, BTN_RB = 5, BTN_BACK = 6, BTN_START = 7,
    AXIS_LX = 0, AXIS_LY = 1, AXIS_LT = 2,
    AXIS_RX = 3, AXIS_RY = 4, AXIS_RT = 5
};

/* ===================== 构造 / 析构 ===================== */

IOSDK::IOSDK() : _isCalibrated(false) {
    _calibOffset.fill(0.0f);
    _activeLegs = {0, 1, 2, 3};
    _node = std::make_shared<rclcpp::Node>("qr_guide_io");

    std::cout << "[IOSDK] 初始化完成\n";
    std::cout << "[提示] 手动调整至趴下姿态，按 START 完成校准\n";

    for (const auto& port : SERIAL_PORTS) {
        _serials.push_back(new SerialPort(
            port, 16, 4000000, 2000, BlockYN::YES,
            bytesize_t::eightbits, parity_t::parity_none,
            stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none
        ));
    }

    for (int i = 0; i < 12; ++i) {
        _motorCmd[i].motorType = MotorType::GO_M8010_6;
        _motorCmd[i].mode = static_cast<unsigned short>(MotorMode::FOC);
        _motorCmd[i].hex_len = 23;

        _motorData[i].motorType = MotorType::GO_M8010_6;
        _motorData[i].hex_len = 31;
    }

#ifdef COMPILE_WITH_MOVE_BASE
    _jointPub = _node->create_publisher<sensor_msgs::msg::JointState>("/real_robot/joint_states", 10);
    _jointState.name = {
        "FR_hip","FR_thigh","FR_calf",
        "FL_hip","FL_thigh","FL_calf",
        "RR_hip","RR_thigh","RR_calf",
        "RL_hip","RL_thigh","RL_calf"
    };
    _jointState.position.resize(12);
#endif

    _joySub = _node->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        rclcpp::QoS(2000),
        std::bind(&IOSDK::joyCallback, this, std::placeholders::_1)
    );

    _currentUserCmd = UserCommand::NONE;
    _currentUserValue = {};
    _lastCalibPromptTime =
        std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}

IOSDK::~IOSDK() {
    for (auto* s : _serials) delete s;
}

/* ===================== Joy 回调 ===================== */

void IOSDK::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    _currentUserCmd = UserCommand::NONE;

    const bool validBtn = msg->buttons.size() >= 8;
    const bool validAx  = msg->axes.size() >= 6;

    if (!_isCalibrated) {
        if (validBtn && msg->buttons[BTN_START]) {
            _currentUserCmd = UserCommand::L1_X;
        } else {
            static int cnt = 0;
            if (++cnt % 200 == 0)
                std::cout << "[IOSDK] 未校准，请按 START\n";
        }
        return;
    }

    if (validBtn && validAx) {
        const bool L1 = msg->buttons[BTN_LB];
        const bool A  = msg->buttons[BTN_A];
        const bool B  = msg->buttons[BTN_B];
        const bool X  = msg->buttons[BTN_X];
        const bool Y  = msg->buttons[BTN_Y];
        const bool ST = msg->buttons[BTN_START];
        const bool L2 = msg->axes[AXIS_LT] > 0.0f;

        if (L2 && B)      _currentUserCmd = UserCommand::L2_B;
        else if (L2 && A) _currentUserCmd = UserCommand::L2_A;
        else if (L2 && X) _currentUserCmd = UserCommand::L2_X;
        else if (L2 && Y) _currentUserCmd = UserCommand::L2_Y;
        else if (ST)      _currentUserCmd = UserCommand::L1_X;
        else if (L1 && A) _currentUserCmd = UserCommand::L1_A;
        else if (L1 && Y) _currentUserCmd = UserCommand::L1_Y;

        _currentUserValue.L2 =
            killZeroOffset((msg->axes[AXIS_LT] + 1.0f) * 0.5f, 0.08f);
        _currentUserValue.lx = killZeroOffset(msg->axes[AXIS_LX], 0.08f);
        _currentUserValue.ly = killZeroOffset(msg->axes[AXIS_LY], 0.08f);
        _currentUserValue.rx = killZeroOffset(msg->axes[AXIS_RX], 0.08f);
        _currentUserValue.ry = killZeroOffset(msg->axes[AXIS_RY], 0.08f);
    }
}

/* ===================== 核心通信 ===================== */

void IOSDK::sendRecv(const UserLowlevel::LowlevelCmd* cmd,
                     LowlevelState* state) {
    if (!cmd || !state || _serials.size() != 4) return;

    state->userCmd   = _currentUserCmd;
    state->userValue = _currentUserValue;
    rclcpp::spin_some(_node);

    /* -------- 校准 -------- */
    if (state->userCmd == UserCommand::L1_X && !_isCalibrated) {
        for (int leg = 0; leg < 4; ++leg) {
            const bool left = (leg == 1 || leg == 3);
            const float sign = left ? -1.0f : 1.0f;

            for (int j = 0; j < 3; ++j) {
                const int id = leg * 3 + j;
                const float gear = (j == 2) ? CALF_TOTAL_GEAR : BASE_GEAR_RATIO;
                const float q = _motorData[id].q / gear;

                float target = 0.0f;
                if (j == 1)      target = sign * (-160.0f * M_PI / 180.0f);
                else if (j == 2) target = sign * (-70.0f * M_PI / 180.0f);
                else {
                    const float hip = 19.0f * M_PI / 180.0f;
                    target = ((leg < 2) ? -sign : sign) * hip;
                }

                _calibOffset[id] = q - target;
            }
        }
        _isCalibrated = true;
        std::cout << "[IOSDK] 校准完成\n";
    }

    /* -------- 电机通信 -------- */
    for (int leg = 0; leg < 4; ++leg) {
        SerialPort* serial = _serials[leg];
        if (!serial) continue;

        for (int j = 0; j < 3; ++j) {
            const int id = leg * 3 + j;
            const float gear = (j == 2) ? CALF_TOTAL_GEAR : BASE_GEAR_RATIO;

            auto& mc = _motorCmd[id];
            auto& md = _motorData[id];
            auto& ms = state->motorState[id];
            const auto& uc = cmd->motorCmd[id];

            const float qCmd =
                _isCalibrated ? (uc.q + _calibOffset[id]) : uc.q;

            mc.id   = j;
            mc.q    = qCmd * gear;
            mc.dq   = uc.dq * gear;
            mc.kp   = uc.Kp;
            mc.kd   = uc.Kd;
            mc.tau  = uc.tau;
            mc.mode = uc.mode;

            if (serial->sendRecv(&mc, &md)) {
                const float qFb = md.q / gear;
                ms.q = _isCalibrated ? (qFb - _calibOffset[id]) : qFb;
                ms.dq = md.dq / gear;
                ms.tauEst = md.tau;
                ms.temp   = md.temp;
                ms.fault  = md.merror;
            } else {
                ms = {};
                ms.fault = 0xFF;
            }
        }
    }

#ifdef COMPILE_WITH_MOVE_BASE
    _jointState.header.stamp = _node->get_clock()->now();
    for (int i = 0; i < 12; ++i)
        _jointState.position[i] = state->motorState[i].q;
    _jointPub.publish(_jointState);
#endif
}

#endif  // COMPILE_WITH_REAL_ROBOT
