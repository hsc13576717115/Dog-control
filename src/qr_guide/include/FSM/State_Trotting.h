#pragma once
#include "FSM/FSMState.h"
#include "interface/IOSDK.h"

/**
 * @brief Trotting状态类 - 实现四足机器人的对角步态控制
 * 
 * 该类负责实现四足机器人的trotting步态，包括：
 * - 摇杆输入处理和滤波
 * - 脚部轨迹规划（摆线运动）
 * - 逆运动学计算
 * - 安全保护机制
 */
class State_Trotting : public FSMState {
public:
    explicit State_Trotting(CtrlComponents *ctrlComp);
    ~State_Trotting() = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;
    void calcTau() {}

    // 闭式解姿态估计相关成员（保持原有功能）
    static constexpr double ANCHOR_UPDATE_T = 0.002;   // 500 Hz更新频率
    Vec3 _estRPY{0,0,0};          // 估计的滚转俯仰偏航角度（弧度）
    Vec3 _targetRPY{0,0,0};       // 期望的水平姿态
    Vec3 _stabP{2.0,2.0,0};      // PID比例增益
    Vec3 _stabD{0.05f,0.05f,0};  // PID微分增益
    Vec3 _errInt{0,0,0};          // 积分项
    double _lastAnchorT = 0.0;

    void updateClosedFormAttitude(double masterT);   // 更新姿态估计
    Vec3 computeStabCorrection();                    // 计算稳定补偿

private:
    // 运动控制参数
    struct MotionParams {
        double stepLenX = 0.0;      // 前后步长
        double stepLenY = 0.0;      // 左右步长
        double omega = 0.0;         // 旋转角速度
        Vec2 joy{0, 0};            // 摇杆输入
        double cycleT;             // 当前周期
    } _motionParams;

    // 滤波参数
    struct FilterParams {
        double stepLenXFiltered = 0.0;  // 滤波后前后步长
        double stepLenYFiltered = 0.0;  // 滤波后左右步长
        double omegaFiltered = 0.0;     // 滤波后旋转速度
        double lastJoyX = 0.0;          // 上一帧左摇杆X
        double lastJoyY = 0.0;          // 上一帧左摇杆Y
        double lastJoyR = 0.0;          // 上一帧右摇杆X
    } _filterParams;

    // 限制参数
    struct LimitParams {
        double maxAccelX = 0.08;        // 最大前后加速度 (m/s²)
        double maxAccelY = 1.0;         // 最大左右加速度 (m/s²)
        double filterAlpha = 0.15;      // 一阶低通滤波系数
    } _limitParams;

    // 加速度限制参数
    struct AccelLimitParams {
        double lastStepLenX = 0.0;      // 上一帧前后步长
        double lastStepLenY = 0.0;      // 上一帧左右步长
        double lastOmega = 0.0;         // 上一帧旋转速度
    } _accelLimitParams;

    // 方向稳定参数
    struct YawStabilityParams {
        double currentYaw = 0.0;        // 当前偏航角
        double targetYaw = 0.0;         // 期望偏航角
        double yawCorrection = 0.0;     // 偏航角修正值
        double yawIntegral = 0.0;       // 偏航角积分项
        double lastYawError = 0.0;      // 上一帧偏航角误差
        bool yawCorrectionEnabled = false;  // 是否启用方向校正
    } _yawStabilityParams;

    // 机械参数
    static constexpr double MAX_SWING_X = 0.50;   // 前后最大步长
    static constexpr double MAX_SWING_Y = 0.1;    // 左右最大步长
    static constexpr double MAX_OMEGA   = 2.0;    // 最大旋转速度

    // 机械几何参数
    static constexpr double SWING_LENGTH = 0.14;
    static constexpr double LIFT_H       = 0.09;
    static constexpr double CYCLE_T      = 0.4;
    static constexpr double L0 = 0.108;
    static constexpr double L1 = 0.225;
    static constexpr double L2 = 0.255;

    // 关节限制参数
    static constexpr double Q0_LIMIT_MIN = -2.60;
    static constexpr double Q0_LIMIT_MAX =  2.60;
    static constexpr double Q1_LIMIT_MIN = -6.50;
    static constexpr double Q1_LIMIT_MAX =  6.50;
    static constexpr double Q2_LIMIT_MIN = -2.30;
    static constexpr double Q2_LIMIT_MAX =  2.30;
    static constexpr double HIP_JOINT_FIXED = 0.0;

    // 腿部相位配置
    static constexpr double LEG_PHASE[4] = {0.0, 0.5, 0.5, 0.0};

    // 内部数据
    Vec12 _initMotorQ{};
    Vec3  _initFootPos[4];
    Vec3  _lastLegQ[4];
    Vec3  _calibOff;
    bool  _isCalibrated = false;
    int   _transitionCount = 0;
    double _startTime = 0;
    IOSDK* _iosdk = nullptr;

    // 工具函数声明
    static double getTimeSec();
    static Vec3 cycloidTraj(double phase);
    Vec3 cycloidTraj3D(double phase) const;
    Vec3 yawCycloidTraj3D(int leg, double phase, bool swing) const;

    void recordFootTrajectory(const Vec3& pl, const Vec3& ac_rel,
                              const Vec3& ac_abs, double t, int leg);
    void printFootStatus(double t, const Vec3& pl, const Vec3& ac_rel,
                         const Vec3& ac_abs, int leg);

    // 私有辅助函数
    void processJoystickInput();
    void applyAccelerationLimits(double& stepLenX, double& stepLenY, double& omega, double dt);
    void generateLegTrajectory(int leg, double masterT, double trans, Vec12& cmd);
    void calculateIKAndApply(int leg, const Vec3& tgtAbs, Vec12& cmd);
    Vec3 clampJointAngles(const Vec3& angles) const;
    Vec3 applyCalibrationOffset(const Vec3& angles, int leg, bool add) const;
};