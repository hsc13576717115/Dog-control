#ifndef QR_GUIDE_INPUT_JOYSTICKMAPPER_H
#define QR_GUIDE_INPUT_JOYSTICKMAPPER_H

#include <sensor_msgs/msg/joy.hpp>

#include "interface/CmdPanel.h"

namespace qr_guide {

// 手柄解析结果：离散命令和连续摇杆值分开保存。
struct UserInput {
    UserCommand command = UserCommand::NONE;
    UserValue value;
};

// 负责把 ROS2 Joy 消息映射成控制器内部使用的 UserCommand / UserValue。
class JoystickMapper {
public:
    static UserInput Map(const sensor_msgs::msg::Joy& joy_msg, bool is_calibrated);
};

}  // namespace qr_guide

#endif  // QR_GUIDE_INPUT_JOYSTICKMAPPER_H
