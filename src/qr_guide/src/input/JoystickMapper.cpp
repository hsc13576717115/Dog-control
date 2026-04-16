#include "input/JoystickMapper.h"

#include "common/mathTools.h"

namespace qr_guide {

namespace {

// 这里约定当前手柄消息的轴和按键索引，和 event2joy.py 输出保持一致。
enum Xbox360JoyMap {
    BTN_A = 0,
    BTN_B = 1,
    BTN_X = 2,
    BTN_Y = 3,
    BTN_LB = 4,
    BTN_RB = 5,
    BTN_BACK = 6,
    BTN_START = 7,
    AXIS_LX = 0,
    AXIS_LY = 1,
    AXIS_LT = 2,
    AXIS_RX = 3,
    AXIS_RY = 4,
    AXIS_RT = 5,
};

}  // namespace

UserInput JoystickMapper::Map(const sensor_msgs::msg::Joy& joy_msg, bool is_calibrated) {
    UserInput input;
    const bool valid_buttons = joy_msg.buttons.size() >= 8;
    const bool valid_axes = joy_msg.axes.size() >= 6;

    if (!valid_buttons || !valid_axes) {
        // 输入长度不对时直接忽略，避免访问越界。
        return input;
    }

    if (!is_calibrated) {
        // 未校准前仅允许 START 触发校准，避免误进控制状态。
        if (joy_msg.buttons[BTN_START]) {
            input.command = UserCommand::L1_X;
        }
        return input;
    }

    const bool a = joy_msg.buttons[BTN_A];
    const bool b = joy_msg.buttons[BTN_B];
    const bool x = joy_msg.buttons[BTN_X];
    const bool y = joy_msg.buttons[BTN_Y];
    const bool l1 = joy_msg.buttons[BTN_LB];
    const bool start = joy_msg.buttons[BTN_START];
    const bool l2_pressed = joy_msg.axes[AXIS_LT] > 0.0f;

    // 离散状态切换命令。
    if (l2_pressed && b) {
        input.command = UserCommand::L2_B;
    } else if (l2_pressed && a) {
        input.command = UserCommand::L2_A;
    } else if (l2_pressed && x) {
        input.command = UserCommand::L2_X;
    } else if (l2_pressed && y) {
        input.command = UserCommand::L2_Y;
    } else if (start) {
        input.command = UserCommand::L1_X;
    } else if (l1 && a) {
        input.command = UserCommand::L1_A;
    } else if (l1 && y) {
        input.command = UserCommand::L1_Y;
    }

    // 连续摇杆值用于 trotting 等状态下的速度指令。
    // 这里统一做死区裁剪，避免手柄回中抖动被当成慢速移动命令。
    input.value.L2 = killZeroOffset((joy_msg.axes[AXIS_LT] + 1.0f) * 0.5f, 0.08f);
    input.value.lx = killZeroOffset(joy_msg.axes[AXIS_LX], 0.08f);
    input.value.ly = killZeroOffset(joy_msg.axes[AXIS_LY], 0.08f);
    input.value.rx = killZeroOffset(joy_msg.axes[AXIS_RX], 0.08f);
    input.value.ry = killZeroOffset(joy_msg.axes[AXIS_RY], 0.08f);
    return input;
}

}  // namespace qr_guide
