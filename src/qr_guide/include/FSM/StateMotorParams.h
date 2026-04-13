#ifndef QR_GUIDE_FSM_STATEMOTORPARAMS_H
#define QR_GUIDE_FSM_STATEMOTORPARAMS_H

#include <array>

#include "message/LowlevelCmd.h"

namespace fsm_motor_params {

// 单关节控制参数。
// 这组参数会被直接拷贝到 UserLowlevel::MotorCmd，对应真实电机控制接口。
// 字段说明：
// - mode: 控制模式，常用的是 DISABLE 或 COMPOUND
// - dq:   目标速度，当前项目里多数状态直接设为 0，主要靠位置环和阻尼工作
// - kp:   位置环刚度，越大通常“站得更硬”，但过大更容易抖动或激发机械共振
// - kd:   速度环阻尼，越大通常“更稳更黏”，但过大也可能带来发热或动作发闷
// - tau:  前馈力矩，常用于补一点支撑力或动作阶段的额外推力
struct JointCommandProfile {
    unsigned int mode;
    float dq;
    float kp;
    float kd;
    float tau;
};

// 单条腿 3 个关节的参数，顺序固定为：
// 0: hip   髋关节
// 1: thigh 大腿关节
// 2: calf  小腿关节
using LegCommandProfile = std::array<JointCommandProfile, 3>;

// 只关心增益和前馈力矩的一组轻量参数。
// StepTest 这种状态里，位置 q 是实时算出来的，所以把 kp / kd / tau 单独抽出来更方便。
struct GainSet {
    float kp;
    float kd;
    float tau;
};

// 常用控制模式别名，集中放在这里方便状态机统一引用。
constexpr unsigned int kDisableMode = static_cast<unsigned int>(ControlMode::DISABLE);
constexpr unsigned int kCompoundMode = static_cast<unsigned int>(ControlMode::COMPOUND);

// Passive 状态参数。
// 设计意图：
// - 不走位置环，因此 kp=0，q 只是上层会额外写成 0
// - hip 的 kd 设得很大，主要是给髋关节更强阻尼，避免完全“泄掉”
// - thigh / calf 的 kd 较小，保留一点阻尼帮助收腿和落地缓冲
// 调参建议：
// - 如果 passive 状态太“软塌塌”，可先适当增大 hip 的 kd
// - 如果 passive 状态拖拽感太强、手动摆腿费劲，就把对应 kd 降低
constexpr LegCommandProfile kPassiveProfile = {{
    {kDisableMode, 0.0f, 0.0f, 300.0f, 0.0f},
    {kDisableMode, 0.0f, 0.0f, 3.0f, 0.0f},
    {kDisableMode, 0.0f, 0.0f, 3.0f, 0.0f},
}};

// FixedStand 状态参数。
// 设计意图：
// - 四条腿统一使用一套“真实机器人站立”参数
// - dq=0, tau=0，主要靠位置环把腿稳定在逆解出来的站立关节角上
// 调参建议：
// - 站立发软：先小幅增加 kp
// - 站立有轻微振荡：优先小幅增加 kd，或者适度减小 kp
// - 三个关节目前用了同一套参数，后续如果你想分关节精调，可直接改成不同值
constexpr LegCommandProfile kFixedStandProfile = {{
    {kCompoundMode, 0.0f, 6.3f, 0.2f, 0.0f},
    {kCompoundMode, 0.0f, 6.3f, 0.2f, 0.0f},
    {kCompoundMode, 0.0f, 6.3f, 0.2f, 0.0f},
}};

// Trotting 状态参数。
// 设计意图：
// - 与站立相比，仍以位置环为主
// - 额外给一点 tau 前馈，帮助动作切换时更顺一点
// 调参建议：
// - 小跑时跟踪慢、抬腿无力：可小幅增大 kp 或 tau
// - 小跑时发硬、敲腿、抖动：优先减小 kp 或 tau，必要时略增 kd
constexpr LegCommandProfile kTrottingProfile = {{
    {kCompoundMode, 0.0f, 6.3f, 0.2f, 0.05f},
    {kCompoundMode, 0.0f, 6.3f, 0.2f, 0.05f},
    {kCompoundMode, 0.0f, 6.3f, 0.2f, 0.05f},
}};

// StepTest 分阶段参数。
// 这个状态机会经历：下蹲 -> 蹬伸 -> 腾空 -> 落地 -> 回位 -> 退出。
// 所以这里不再定义整条腿 profile，而是按阶段定义增益组。
//
// 阶段含义与调参方向：
// - Squat:   下蹲蓄力阶段，通常希望稳、不要抖
// - Thrust:  蹬伸发力阶段，可适当提高阻尼避免发散
// - Air:     腾空阶段，位置环可稍软一些，保留少量 tau 做姿态/落点辅助
// - Land:    落地阶段，通常需要比 Air 稍强的阻尼和一点额外 tau
// - Return:  回位阶段，让腿从动作末端收回到较稳定的姿态
// - Exit:    退出 StepTest 回到初始姿态时使用
//
// 调参建议：
// - 起跳不明显：优先看 Thrust
// - 空中腿太僵：优先看 Air
// - 落地太冲、太硬：优先看 Land
// - 动作结束回位生硬：优先看 Return / Exit
constexpr GainSet kStepTestSquatGains{6.2f, 0.20f, 0.0f};
constexpr GainSet kStepTestThrustGains{6.2f, 0.25f, 0.0f};
constexpr GainSet kStepTestAirGains{3.0f, 0.35f, 0.05f};
constexpr GainSet kStepTestLandGains{3.2f, 0.30f, 0.08f};
constexpr GainSet kStepTestReturnGains{3.2f, 0.25f, 0.05f};
constexpr GainSet kStepTestExitGains{5.0f, 0.15f, 0.0f};

// 把单关节 profile 写回到电机命令。
// 这个函数只改 mode / dq / kp / kd / tau，不改 q。
// 这样状态机就可以把“参数模板”和“实时轨迹位置 q”分开管理。
inline void ApplyJointProfile(UserLowlevel::MotorCmd* motor_cmd, const JointCommandProfile& profile) {
    motor_cmd->mode = profile.mode;
    motor_cmd->dq = profile.dq;
    motor_cmd->Kp = profile.kp;
    motor_cmd->Kd = profile.kd;
    motor_cmd->tau = profile.tau;
}

// 把一整条腿的 3 个关节参数同时写回。
// 常用于进入某个状态时，先给这条腿整体套上一组默认控制参数。
inline void ApplyLegProfile(UserLowlevel::LowlevelCmd* low_cmd, int leg, const LegCommandProfile& profile) {
    for (int joint = 0; joint < 3; ++joint) {
        ApplyJointProfile(&low_cmd->motorCmd[leg * 3 + joint], profile[joint]);
    }
}

// 只覆写 kp / kd / tau。
// 适合像 StepTest 这种 q 每一拍都在变化、但想按阶段切换增益的场景。
inline void ApplyGainSet(UserLowlevel::MotorCmd* motor_cmd, const GainSet& gains) {
    motor_cmd->Kp = gains.kp;
    motor_cmd->Kd = gains.kd;
    motor_cmd->tau = gains.tau;
}

}  // namespace fsm_motor_params

#endif  // QR_GUIDE_FSM_STATEMOTORPARAMS_H
