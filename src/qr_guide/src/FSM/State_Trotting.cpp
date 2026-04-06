#include "FSM/State_Trotting.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <thread>
#include "common/mathTools.h"   
#include <Eigen/Core>

template<typename T>
const T& clamp(const T& v, const T& min, const T& max) {
    return std::min(std::max(v, min), max);
}

extern Vec3 fkCheckAxis(const Vec3 &qUser, int leg);
Vec3 ikCheckAxis(const Vec3 &pDes, int leg);

/* ======================== 静态工具函数 ======================== */

double State_Trotting::getTimeSec() {
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    duration<double> sec = steady_clock::now() - start;
    return sec.count();
}

Vec3 State_Trotting::cycloidTraj(double phase) {
    const double theta = phase * 4.0 * M_PI;
    const double x = (theta <= 2.0 * M_PI) ?
                     SWING_LENGTH * (theta - sin(theta)) / (2.0 * M_PI) :
                     SWING_LENGTH * (1.0 - (theta - 2.0 * M_PI) / (2.0 * M_PI));
    const double z = (theta <= 2.0 * M_PI) ?
                     LIFT_H * (1.0 - cos(theta)) / 2.0 : 0.0;
    return Vec3(x - 0.5 * SWING_LENGTH, 0.0, z);
}

/* ======================== 轨迹生成函数 ======================== */

Vec3 State_Trotting::cycloidTraj3D(double phase) const {
    const double theta = phase * 4.0 * M_PI;
    const bool swing = (theta <= 2.0 * M_PI);

    // 前后位移计算
    double alongX = 0.0;
    if (std::fabs(_motionParams.stepLenX) > 1e-6) {
        double prog = swing ? (theta - sin(theta)) / (2.0 * M_PI)
                            : (1.0 - (theta - 2.0 * M_PI) / (2.0 * M_PI));
        alongX = _motionParams.stepLenX * (prog - 0.5);
    }

    // 左右位移计算
    double alongY = 0.0;
    if (std::fabs(_motionParams.stepLenY) > 1e-6) {
        double prog = swing ? (theta - sin(theta)) / (2.0 * M_PI)
                            : (1.0 - (theta - 2.0 * M_PI) / (2.0 * M_PI));
        alongY = _motionParams.stepLenY * (prog - 0.5);
    }

    // 高度计算：只要有平移或旋转就抬腿
    double z = 0.0;
    if (swing && (std::fabs(_motionParams.stepLenX) > 1e-6 ||
                  std::fabs(_motionParams.stepLenY) > 1e-6 ||
                  std::fabs(_motionParams.omega) > 1e-3))
        z = LIFT_H * (1.0 - cos(theta)) / 2.0;

    return Vec3(alongX, alongY, z);
}

Vec3 State_Trotting::yawCycloidTraj3D(int leg,
                                      double phase,
                                      bool swing) const {
    if (!swing) {
        return Vec3::Zero();  // 🔥 支撑腿：绝对不动
    }

    const double theta = phase * 4.0 * M_PI;

    // 半个周期内完成的旋转角
    const double yawTotal = _motionParams.omega * CYCLE_T * 0.5;

    // cycloid 相位进度（0 → 1）
    double prog = (theta - sin(theta)) / (2.0 * M_PI);
    double yawProg = yawTotal * prog;

    // 以质心为中心旋转初始足端
    const Vec3& p0 = _initFootPos[leg];

    double c = cos(yawProg);
    double s = sin(yawProg);

    Vec3 p;
    p.x() = c * p0.x() - s * p0.y();
    p.y() = s * p0.x() + c * p0.y();
    p.z() = p0.z();

    // 抬腿
    p.z() += LIFT_H * (1.0 - cos(theta)) / 2.0;

    return p - p0;
}



/* ======================== 数据记录与打印函数 ======================== */

void State_Trotting::recordFootTrajectory(const Vec3& pl, const Vec3& ac_rel,
                                          const Vec3& ac_abs, double t, int leg) {
    static std::ofstream csv;
    static bool init = false;
    if (!init) {
        csv.open("/home/orangepi/qr_ws/foot_trajectory_comparison.csv",
                 std::ios::out | std::ios::trunc);
        if (csv.is_open())
            csv << "time,leg,pl_x,pl_y,pl_z,ac_rel_x,ac_rel_y,ac_rel_z\n";
        init = true;
    }
    if (csv.is_open()) {
        csv << std::fixed << std::setprecision(4)
            << t << "," << leg << ","
            << pl.x() << "," << pl.y() << "," << pl.z() << ","
            << ac_rel.x() << "," << ac_rel.y() << "," << ac_rel.z() << "\n";
        csv.flush();
    }
}

void State_Trotting::printFootStatus(double t, const Vec3& pl, const Vec3& ac_rel,
                                     const Vec3& ac_abs, int leg) {
    std::cout << std::fixed << std::setprecision(3)
              << "[Trot] leg=" << leg
              << " t=" << std::setw(6) << t
              << " | plan(rel) X=" << std::setw(6) << pl.x()
              << " Y=" << std::setw(6) << pl.y()
              << " Z=" << std::setw(6) << pl.z()
              << " | 摇杆:(" << _motionParams.joy.x() << "," << _motionParams.joy.y()
              << ")  omega=" << _motionParams.omega << "\n";
}

/* ======================== 构造函数与生命周期管理 ======================== */

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"),
      _transitionCount(0), _isCalibrated(false) {
    _calibOff.setZero();
    
    // 初始化滤波器参数
    _filterParams = {};
    _limitParams.maxAccelX = 0.25;  // 增加最大前后加速度限制
    _limitParams.maxAccelY = 0.3;   // 增加最大左右加速度限制
    _limitParams.filterAlpha = 0.15;
    
    // 初始化加速度限制参数
    _accelLimitParams = {};
    
    // 初始化方向稳定参数
    _yawStabilityParams = {};
}

void State_Trotting::enter() {
    std::cout << "\n=====================================\n";
    std::cout << "[Trot] 四腿对角步态 —— 简化版（无IMU辅助）\n";
    std::cout << "=====================================\n";

    // 初始化电机位置
    for (int i = 0; i < 12; ++i) 
        _initMotorQ[i] = _lowState->motorState[i].q;
    
    // 初始化脚部位置
    for (int leg = 0; leg < 4; ++leg) {
        Vec3 q(_initMotorQ[leg*3], _initMotorQ[leg*3+1], _initMotorQ[leg*3+2]);
        _lastLegQ[leg] = q;
        _initFootPos[leg] = fkCheckAxis(q, leg);
    }

    // 初始化IO接口
    IOSDK* iosdk = dynamic_cast<IOSDK*>(_ctrlComp->ioInter);
    if (iosdk && iosdk->isCalibrated()) {
        for (int m = 0; m < 3; ++m) 
            _calibOff(m) = iosdk->getCalibOffset()[1*3 + m];
        _isCalibrated = true;
    } else {
        _calibOff.setZero();
        _isCalibrated = false;
    }
    _iosdk = iosdk;

    // 设置电机控制模式
    for (int leg = 0; leg < 4; ++leg) {
        for (int j = 0; j < 3; ++j) {
            int id = leg * 3 + j;
            auto& motorCmd = _lowCmd->motorCmd[id];
            motorCmd.mode = 1;
            motorCmd.dq   = 0;
            motorCmd.tau  = 0.05f;
            motorCmd.Kp   = 6.2f;
            motorCmd.Kd   = 0.2f;
            motorCmd.q    = _lastLegQ[leg](j);
        }
    }

    _startTime = getTimeSec();
    _transitionCount = 0;
    _motionParams = {};  // 重置运动参数
    
    // 重置滤波器状态
    _filterParams = {};
    _accelLimitParams = {};
    
    // 重置方向稳定参数
    _yawStabilityParams = {};
}

/* ======================== 核心控制逻辑 ======================== */

void State_Trotting::processJoystickInput() {
    if (_iosdk == nullptr) {
        _motionParams = {};
        _yawStabilityParams = {};
        return;
    }

    // 获取摇杆原始值
    double lx = static_cast<double>(_iosdk->_currentUserValue.lx);
    double ly = static_cast<double>(_iosdk->_currentUserValue.ly);
    double rx = static_cast<double>(_iosdk->_currentUserValue.rx);

    // 计算时间步长（用于加速度限制）
    static double lastTime = getTimeSec();
    double currentTime = getTimeSec();
    double dt = std::max(0.002, std::min(currentTime - lastTime, 0.01));  // 限制dt范围
    lastTime = currentTime;
    
    // 死区处理和映射
    const double RX_DEAD = 0.1;
    double rawOmega = (std::fabs(rx) > RX_DEAD) ?
             std::copysign(std::fabs(rx) - RX_DEAD, rx) / (1.0 - RX_DEAD) * MAX_OMEGA : 0.0;

    const double DEAD_X = 0.1;
    const double DEAD_Y = 0.2;
    double rawStepLenX = (std::fabs(ly) > DEAD_X) ?
                std::copysign((std::fabs(ly) - DEAD_X) / (1.0 - DEAD_X), -ly) * MAX_SWING_X : 0.0;
    double rawStepLenY = (std::fabs(lx) > DEAD_Y) ?
                std::copysign((std::fabs(lx) - DEAD_Y) / (1.0 - DEAD_Y), lx) * MAX_SWING_Y : 0.0;
    
    _motionParams.joy = Vec2(ly, lx);
    
    // 应用加速度限制
    applyAccelerationLimits(rawStepLenX, rawStepLenY, rawOmega, dt);
}

void State_Trotting::applyAccelerationLimits(double& stepLenX, double& stepLenY, double& omega, double dt) {
    // 计算最大允许变化量
    double maxDeltaX = _limitParams.maxAccelX * dt;
    double maxDeltaY = _limitParams.maxAccelY * dt;
    
    // 限制命令变化率
    double targetStepLenX = stepLenX;
    double targetStepLenY = stepLenY;
    double targetOmega = omega;
    
    // 限制X方向变化
    double deltaX = targetStepLenX - _accelLimitParams.lastStepLenX;
    deltaX = std::max(std::min(deltaX, maxDeltaX), -maxDeltaX);
    _motionParams.stepLenX = _accelLimitParams.lastStepLenX + deltaX;
    _accelLimitParams.lastStepLenX = _motionParams.stepLenX;
    
    // 限制Y方向变化
    double deltaY = targetStepLenY - _accelLimitParams.lastStepLenY;
    deltaY = std::max(std::min(deltaY, maxDeltaY), -maxDeltaY);
    _motionParams.stepLenY = _accelLimitParams.lastStepLenY + deltaY;
    _accelLimitParams.lastStepLenY = _motionParams.stepLenY;
    
    // 旋转命令直接应用
    _motionParams.omega = targetOmega;
}

void State_Trotting::generateLegTrajectory(int leg,
                                           double masterT,
                                           double trans,
                                           Vec12& cmd) {
    double legT = std::fmod(masterT + LEG_PHASE[leg] * CYCLE_T, CYCLE_T);
    double ph   = legT / CYCLE_T;
bool swing = (ph < 0.5);

Vec3 tgtRel = Vec3::Zero();

bool hasXY =
    std::fabs(_motionParams.stepLenX) > 1e-6 ||
    std::fabs(_motionParams.stepLenY) > 1e-6;

bool hasYaw = std::fabs(_motionParams.omega) > 1e-3;

/* 平移 */
if (hasXY) {
    tgtRel += cycloidTraj3D(ph);
}

/* 原地旋转：只给摆动腿 */
if (hasYaw && !hasXY) {
    tgtRel += yawCycloidTraj3D(leg, ph, swing);
}

tgtRel *= trans;
Vec3 tgtAbs = _initFootPos[leg] + tgtRel;


    /* ---------- 3️⃣ IK 可达性限制 ---------- */
    double dist = tgtAbs.norm();
    double dMin = std::fabs(L1 - L2);
    double dMax = L1 + L2;
    if (dist < dMin - 1e-3 || dist > dMax + 1e-3)
        tgtAbs = tgtAbs.normalized() *
                 std::min(dMax * 0.9, std::max(dMin * 1.1, dist));

    calculateIKAndApply(leg, tgtAbs, cmd);
}


void State_Trotting::calculateIKAndApply(int leg, const Vec3& tgtAbs, Vec12& cmd) {
    // 逆运动学计算
    Vec3 qDesAxis = ikCheckAxis(tgtAbs, leg);
    Vec3 qRaw = qDesAxis;
    
    // 应用校准偏移
    if (_isCalibrated && leg == 1) 
        qRaw += _calibOff;
    
    // 关节角度限制
    qRaw = clampJointAngles(qRaw);
    
    // 移除校准偏移以获得实际命令
    Vec3 qDes = qRaw;
    if (_isCalibrated && leg == 1) 
        qDes -= _calibOff;

    // 平滑插值
    Vec3 &qLast = _lastLegQ[leg];
    Vec3  delta = qDes - qLast;
    const double MAX_DELTA = 0.03;
    if (delta.norm() > MAX_DELTA) 
        delta = MAX_DELTA * delta.normalized();
    Vec3 qCmd = qLast + delta;
    qLast = qCmd;

    // 设置命令
    int id = leg * 3;
    cmd[id]   = qCmd(0);
    cmd[id+1] = qCmd(1);
    cmd[id+2] = qCmd(2);
}

Vec3 State_Trotting::clampJointAngles(const Vec3& angles) const {
    Vec3 clamped = angles;
    clamped(0) = clamp(clamped(0), Q0_LIMIT_MIN, Q0_LIMIT_MAX);
    clamped(1) = clamp(clamped(1), Q1_LIMIT_MIN, Q1_LIMIT_MAX);
    clamped(2) = clamp(clamped(2), Q2_LIMIT_MIN, Q2_LIMIT_MAX);
    return clamped;
}

Vec3 State_Trotting::applyCalibrationOffset(const Vec3& angles, int leg, bool add) const {
    Vec3 result = angles;
    if (_isCalibrated && leg == 1) {
        if (add) {
            result = result + _calibOff;
        } else {
            result = result - _calibOff;
        }
    }
    return result;
}

void State_Trotting::run() {
    if (_transitionCount < 100) 
        _transitionCount++;

    // 处理摇杆输入
    processJoystickInput();

    double masterT = std::fmod(getTimeSec() - _startTime, CYCLE_T);
    float  trans   = std::min(1.0f, (float)_transitionCount / 100.0f);
    Vec12  cmd     = _initMotorQ;

    // 更新姿态估计（保持原有功能）
    // updateClosedFormAttitude(masterT);

    // 为每条腿生成轨迹
    for (int leg = 0; leg < 4; ++leg) {
        generateLegTrajectory(leg, masterT, trans, cmd);

        // 打印调试信息（仅第一条腿）
        if (leg == 0) {
            Vec3 qAct(_lowState->motorState[leg*3].q,
                      _lowState->motorState[leg*3+1].q,
                      _lowState->motorState[leg*3+2].q);
            Vec3 actualAbs = fkCheckAxis(qAct, leg);
            Vec3 actualRel = actualAbs - _initFootPos[leg];
            // printFootStatus(masterT, tgtRel, actualRel, actualAbs, leg);
            // recordFootTrajectory(tgtRel, actualRel, actualAbs, masterT, leg);
        }
    }
    
    _lowCmd->setQ(cmd);
}

void State_Trotting::exit() {
    std::cout << "\n[Trot] 退出：平滑回初始位置...\n";
    for (int i = 0; i < 50; ++i) {
        Vec12 tmp = _initMotorQ;
        for (int leg = 0; leg < 4; ++leg) {
            int id = leg * 3;
            tmp[id]   = HIP_JOINT_FIXED;
            tmp[id+1] = tmp[id+1] * 0.9 + _initMotorQ[id+1] * 0.1;
            tmp[id+2] = tmp[id+2] * 0.9 + _initMotorQ[id+2] * 0.1;
        }
        _lowCmd->setQ(tmp);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    _lowCmd->setQ(_initMotorQ);
    std::cout << "[Trot] 已回到初始位置\n";
}

FSMStateName State_Trotting::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B) 
        return FSMStateName::PASSIVE;
    if (_lowState->userCmd == UserCommand::L2_A) 
        return FSMStateName::FIXEDSTAND;
    if (_lowState->userCmd == UserCommand::L2_Y) 
        return FSMStateName::STEPTEST;
    return FSMStateName::TROTTING;
}

// 静态常量定义
constexpr double State_Trotting::LEG_PHASE[4];

// 闭式解姿态估计函数（保持原有功能）
void State_Trotting::updateClosedFormAttitude(double masterT) {
    /* 1. 根据当前主相位决定"支撑对" */
    bool lf_rh_support = (std::fmod(masterT, CYCLE_T) < 0.5 * CYCLE_T);
    int leg0 = lf_rh_support ? 0 : 1;   // LF 或 RF
    int leg2 = lf_rh_support ? 2 : 3;   // RH 或 LH

    /* 2. 用正确的支撑腿做 FK */
    Vec3 p0 = fkCheckAxis(Vec3(_lowState->motorState[leg0*3].q,
                               _lowState->motorState[leg0*3+1].q,
                               _lowState->motorState[leg0*3+2].q), leg0);
    Vec3 p2 = fkCheckAxis(Vec3(_lowState->motorState[leg2*3].q,
                               _lowState->motorState[leg2*3+1].q,
                               _lowState->motorState[leg2*3+2].q), leg2);

    /* 3. 其余不变 */
    RotMat R = _lowState->getRotMat();
    Vec3   rpy = rotMatToRPY(R);
    Vec2 d = Vec2(p2.x() - p0.x(), p2.y() - p0.y());
    rpy(2) = std::atan2(d.y(), d.x());
    _estRPY = rpy;
}

// 自稳补偿函数（保持原有功能）
Vec3 State_Trotting::computeStabCorrection() {
    Vec3 err = _targetRPY - _estRPY;

    /* 1. 角度死区：0.3° 以内认为是噪声/足底弹性 */
    const float DEAD_BAND = 2.0f * M_PI / 180.0f;   // 0.3°
    for (int i = 0; i < 2; ++i) {
        if (std::fabs(err[i]) < DEAD_BAND)  err[i] = 0.0f;
    }

    /* 2. 积分抗饱和（毫米级） */
    _errInt += err * ANCHOR_UPDATE_T;
    _errInt = _errInt.cwiseMin(0.03f).cwiseMax(-0.03f);

    /* 3. 高增益 PID → 输出亚毫米封顶 */
    Vec3 out = _stabP.cwiseProduct(err) +
               _stabD.cwiseProduct(-_lowState->getGyroGlobal());

    /* 4. 单帧硬限幅：±0.2 mm（500 Hz 下肉眼无感） */
    out = out.cwiseMin(0.01f).cwiseMax(-0.01f);

    /* 5. 一阶低通（可选，更丝滑） */
    static Vec3 outFilt = Vec3::Zero();
    const float alpha = 0.15f;          // 越小越平滑
    outFilt = alpha * out + (1 - alpha) * outFilt;

    printf("corr=%.3f %.3f\n", outFilt.x(), outFilt.y());
    printf("err=%.4f %.4f  dead=%.4f\n", err.x(), err.y(), DEAD_BAND);
    return outFilt;
}
