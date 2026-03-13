#include "FSM/State_StepTest.h"
#include "FSM/State_FixedStand.h"
#include "interface/IOSDK.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <cstdlib>

/* ================== 跳跃功能可调参数 ================== */
static constexpr double SQUAT_DEPTH    = 0.15;    // 下蹲深度（m） - 蓄力阶段向下移动距离
static constexpr double JUMP_HEIGHT    = 0.14;    // 目标跳跃高度（m） - 相对于初始位置的最大抬升
static constexpr double MAX_X_FORWARD  = 0.30;    // 最大前冲距离（m） - 爆发阶段向前移动产生推力
static constexpr double LANDING_OFFSET = 0.08;    // 落地时足端前移补偿（m）- 确保足端在身体前方缓冲
static constexpr double JUMP_CYCLE_T   = 3.5;     // 完整跳跃周期（s）：下蹲(3.0s)→起跳(0.2s)→空中(0.2s)→落地(0.3s)
static constexpr int    TRANSITION_DURATION = 50;
static const std::string CSV_PATH = "/home/orangepi/catkin_ws/jump_trajectory.csv";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template<typename T>
const T& clamp(const T& value, const T& min_val, const T& max_val) {
    return std::min(std::max(value, min_val), max_val);
}

extern Vec3 fkCheckAxis(const Vec3 &qUser, int leg);
Vec3 ikCheckAxis(const Vec3 &pDes, int leg);

/* -------------- 时间工具 -------------- */
static double getTimeSec() {
    using namespace std::chrono;
    static const steady_clock::time_point start = steady_clock::now();
    duration<double> sec = steady_clock::now() - start;
    return sec.count();
}

/* -------------- CSV记录 -------------- */
static void recordJumpTrajectory(const Vec3& planned_rel, const Vec3& actual_rel,
                                 const Vec3& actual_abs, double t, int leg) {
    static std::ofstream csv_file;
    static bool is_init = false;
    if (!is_init) {
        csv_file.open(CSV_PATH, std::ios::out | std::ios::trunc);
        if (csv_file.is_open()) {
            csv_file << "time_sec,leg_id,planned_x_rel_m,planned_y_rel_m,planned_z_rel_m,"
                     << "actual_x_rel_m,actual_y_rel_m,actual_z_rel_m,"
                     << "actual_x_abs_m,actual_y_abs_m,actual_z_abs_m\n";
            std::cout << "[INFO] 跳跃轨迹CSV已创建：" << CSV_PATH << "\n";
        } else {
            std::cerr << "[ERROR] 无法打开CSV文件！路径：" << CSV_PATH << "\n";
        }
        is_init = true;
    }
    if (csv_file.is_open()) {
        csv_file << std::fixed << std::setprecision(6)
                 << t << "," << leg << ","
                 << planned_rel.x() << "," << planned_rel.y() << "," << planned_rel.z() << ","
                 << actual_rel.x() << "," << actual_rel.y() << "," << actual_rel.z() << ","
                 << actual_abs.x() << "," << actual_abs.y() << "," << actual_abs.z() << "\n";
        csv_file.flush();
    }
}

/* -------------- 控制台打印 -------------- */
static void printJumpStatus(double t, const Vec3& planned_rel, const Vec3& actual_rel,
                            const Vec3& actual_abs, int leg) {
    if (leg == 0) {
        std::cout << std::fixed << std::setprecision(3)
                  << "[JumpTest] t=" << std::setw(6) << t << " | "
                  << "计划(rel): X=" << std::setw(6) << planned_rel.x() << " Z=" << std::setw(6) << planned_rel.z() << " | "
                  << "实际(rel): X=" << std::setw(6) << actual_rel.x() << " Z=" << std::setw(6) << actual_rel.z() << "\n";
    }
}

static Vec3 jumpTraj(double t_cycle)
{
    Vec3 rel_pos(0.0, 0.0, 0.0);
    double x = 0.0, z = 0.0;

    if (t_cycle < 3.00) {                                    // ① 蓄力 3.00 s - 足端偏前
        double prog = t_cycle / 3.00;
        double theta = prog * M_PI;
        z = SQUAT_DEPTH * (1.0 - cos(theta)) * 0.5;         // 平滑下蹲
        x = 0.05 * sin(prog * M_PI * 0.5);                  // 足端逐渐前移，蓄力准备
    } else if (t_cycle < 3.20) {                              // ② 爆发 0.20 s - 向后下方用力（产生向前推力）
        double prog = (t_cycle - 3.00) / 0.20;
        z = SQUAT_DEPTH - (SQUAT_DEPTH + JUMP_HEIGHT) * pow(prog, 1.2);
        x = -MAX_X_FORWARD * pow(prog, 1.3);                  // 足端向前移动，身体向后倾斜产生向前推力
    } else if (t_cycle < 3.40) {                              // ③ 空中摆动 0.20 s - 快速摆动到落地姿势
        // 在空中快速调整足端位置，准备落地
        double prog = (t_cycle - 3.20) / 0.20;
        x = LANDING_OFFSET * prog;                            // 足端逐渐向前移动到落地位置（相对于初始位置）
        z = JUMP_HEIGHT;                                      // 保持空中高度
    } else {                                                  // ④ 落地缓冲+回正 0.30 s
        double prog = (t_cycle - 3.40) / 0.30;                // 剩余 0.3 s
        if (prog < 0.6) {
            // 落地阶段：足端前移确保在身体前方缓冲
            double land_prog = prog / 0.6;                    // 0~1
            x = LANDING_OFFSET;                               // 足端移动到身体前方
            z = JUMP_HEIGHT * (1.0 - land_prog);              // 从空中高度下降
        } else {
            // 回正阶段：回到初始位置
            double return_prog = (prog - 0.6) / 0.4;          // 0~1
            x = LANDING_OFFSET * (1.0 - return_prog);         // 逐渐回到原位
            z = 0;                                            // 已落地
        }
    }
    rel_pos.x() = x;
    rel_pos.z() = z;
    return rel_pos;
}

/* ================== 分阶段增益（适配优化后的跳跃阶段） ================== */
struct GainSet { float kp, kd, tau; };
static constexpr GainSet
    G_SQUAT  = { 6.2f, 0.20f, 0.0f },   // 蓄力：高Kp保证下蹲位置精准，小tau辅助蓄力
    G_THRUST = { 6.2f, 0.25f, 0.0f },   // 爆发：高Kp保证爆发力，适当Kd控制震荡
    G_AIR    = { 3.0f, 0.35f, 0.05f },  // 空中调整：适中Kp+Kd，快速调整位置
    G_LAND   = { 3.2f, 0.30f, 0.08f },  // 落地缓冲：更高Kp保证快速响应，适中Kd缓冲，小tau保持力矩
    G_RETURN = { 3.2f, 0.25f, 0.05f }; // 回正：高Kp保证快速回到初始位置，中Kd缓冲，小tau保持力矩

/* ============================================== */

/* -------------------- 构造函数 -------------------- */
State_StepTest::State_StepTest(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::STEPTEST, "STEPTEST"),
      _transitionCount(0), _isCalibrated(false), _initMotorQ(12),
      _isJumpCompleted(false),  _isCycleEnded(false) {
    _calibOff.setZero();
    for (int i = 0; i < 4; ++i) {
        _lastLegQ[i].setZero();
        _initFootPos[i].setZero();
    }
}

/* -------------------- enter -------------------- */
void State_StepTest::enter() {
    std::cout << "\n=====================================\n";
    std::cout << "[JumpTest] 优化版跳跃模式（跳一次自动回Trotting）\n";
    std::cout << "[参数] 下蹲15cm | 高度12cm | 前冲15cm | 落地前移8cm | 周期3.5s\n";
    std::cout << "[机械参数] L0=" << L0 << "m, L1=" << L1 << "m, L2=" << L2 << "m\n";
    std::cout << "[操作] L2+B→被动 | L2+A→站立 | L2+X→手动回Trotting\n";
    std::cout << "=====================================\n";

    for (int i = 0; i < 12; ++i) _initMotorQ[i] = _lowState->motorState[i].q;

    for (int leg = 0; leg < 4; ++leg) {
        int leg_base_id = leg * 3;
        Vec3 init_q(HIP_JOINT_FIXED, _initMotorQ[leg_base_id + 1], _initMotorQ[leg_base_id + 2]);
        _lastLegQ[leg] = init_q;
        _initFootPos[leg] = fkCheckAxis(init_q, leg);
        std::cout << "[初始化] 腿" << leg << "：X=" << _initFootPos[leg].x() << " Z=" << _initFootPos[leg].z() << "\n";
    }

    IOSDK* iosdk = dynamic_cast<IOSDK*>(_ctrlComp->ioInter);
    if (iosdk && iosdk->isCalibrated()) {
        _calibOff = Vec3(iosdk->getCalibOffset()[3], iosdk->getCalibOffset()[4], iosdk->getCalibOffset()[5]);
        _isCalibrated = true;
        std::cout << "[校准] 右腿偏移：(" << _calibOff(0) << "," << _calibOff(1) << "," << _calibOff(2) << ")\n";
    } else {
        std::cerr << "[警告] 未校准！可能影响轨迹精度\n";
        _calibOff.setZero();
        _isCalibrated = false;
    }

    float kp = G_SQUAT.kp, kd = G_SQUAT.kd, tau = G_SQUAT.tau;
    std::cout << "[电机] 初始Kp=" << kp << " | Kd=" << kd << " | 力矩=" << tau << "\n";

    for (int leg = 0; leg < 4; ++leg) {
        for (int j = 0; j < 3; ++j) {
            int id = leg * 3 + j;
            _lowCmd->motorCmd[id].mode = 1;
            _lowCmd->motorCmd[id].dq   = 0;
            _lowCmd->motorCmd[id].Kp   = kp;
            _lowCmd->motorCmd[id].Kd   = kd;
            _lowCmd->motorCmd[id].tau  = tau;
            _lowCmd->motorCmd[id].q    = _lastLegQ[leg](j);
        }
    }

    _startTime     = getTimeSec();
    _transitionCount = 0;
    _isJumpCompleted = false;
    _isCycleEnded    = false;
}

/* -------------------- run -------------------- */
void State_StepTest::run() {
    double t_total = getTimeSec() - _startTime;
    double t_cycle = std::fmod(t_total, JUMP_CYCLE_T); // 0~3.5 s

    _transitionCount++;

    if (t_total > JUMP_CYCLE_T && !_isCycleEnded) {
        std::cout << "[JumpTest] 完成1周期，准备回Trotting\n";
        _isCycleEnded = true;
        _isJumpCompleted = true;
    }

    float transition_scale = std::min(1.0f, (float)_transitionCount / TRANSITION_DURATION);

    // 根据当前时间阶段选择合适的增益参数
    GainSet g;
    if (t_cycle < 3.00)      g = G_SQUAT;     // 0-3.0 s 蓄力
    else if (t_cycle < 3.20) g = G_THRUST;    // 3.0-3.2 s 爆发
    else if (t_cycle < 3.40) g = G_AIR;       // 3.2-3.4 s 空中调整
    else if (t_cycle < 3.55) g = G_LAND;      // 3.4-3.55 s 落地缓冲
    else                     g = G_RETURN;    // 3.55-3.5 s 回正

    for (int leg = 0; leg < 4; ++leg) {
        int leg_base_id = leg * 3;
        if (_isCycleEnded) continue;

        Vec3 tgtRel = jumpTraj(t_cycle) * transition_scale;
        Vec3 tgtAbs = _initFootPos[leg] + tgtRel;

        // 可达性检查和修正
        double min_reach = std::fabs(L0 + L1 - L2);
        double max_reach = L0 + L1 + L2;
        double dist = tgtAbs.norm();
        if (dist < min_reach - 0.002 || dist > max_reach + 0.002) {
            std::cerr << "[警告] 腿" << leg << " 不可达！dist=" << dist << "（" << min_reach << "~" << max_reach << "）\n";
            // 限制目标位置在可达范围内，使用自定义的clamp函数
            double target_dist = clamp(dist, min_reach + 0.005, max_reach - 0.005);
            Vec3 direction = tgtAbs.normalized();
            tgtAbs = direction * target_dist;
        }

        Vec3 qActual(_lowState->motorState[leg_base_id].q,
                     _lowState->motorState[leg_base_id + 1].q,
                     _lowState->motorState[leg_base_id + 2].q);
        Vec3 actualAbs = fkCheckAxis(qActual, leg);
        Vec3 actualRel = actualAbs - _initFootPos[leg];

        recordJumpTrajectory(tgtRel, actualRel, actualAbs, t_total, leg);
        printJumpStatus(t_total, tgtRel, actualRel, actualAbs, leg);

        Vec3 qDesAxis = ikCheckAxis(tgtAbs, leg);
        qDesAxis(0) = HIP_JOINT_FIXED;

        // 关节限位检查
        qDesAxis(0) = clamp(qDesAxis(0), Q0_LIMIT_MIN, Q0_LIMIT_MAX);
        qDesAxis(1) = clamp(qDesAxis(1), Q1_LIMIT_MIN, Q1_LIMIT_MAX);
        qDesAxis(2) = clamp(qDesAxis(2), Q2_LIMIT_MIN, Q2_LIMIT_MAX);

        // 平滑插值，防止电机角度突变
        Vec3 delta = qDesAxis - _lastLegQ[leg];
        const double MAX_DELTA = 0.045;
        if (delta.norm() > MAX_DELTA) {
            delta = MAX_DELTA * delta.normalized();
        }
        Vec3 qCmd = _lastLegQ[leg] + delta;
        _lastLegQ[leg] = qCmd;

        // 下发关节命令，使用当前阶段的增益参数
        for (int j = 0; j < 3; ++j) {
            int id                    = leg_base_id + j;
            _lowCmd->motorCmd[id].q   = qCmd(j);
            _lowCmd->motorCmd[id].Kp  = g.kp;
            _lowCmd->motorCmd[id].Kd  = g.kd;
            _lowCmd->motorCmd[id].tau = g.tau;
            
            // 确保电机模式正确
            _lowCmd->motorCmd[id].mode = 1; // 位置控制模式
        }

        // 调试信息输出
        static int db_cnt = 0;
        if (++db_cnt % 50 == 0 && leg == 0) {
            std::cout << "------------------------------------\n";
            printf("[调试] 腿0 下发：Q0=%+7.3f Q1=%+7.3f Q2=%+7.3f\n", qCmd(0), qCmd(1), qCmd(2));
            printf("[调试] 腿0 回读：Q0=%+7.3f Q1=%+7.3f Q2=%+7.3f\n",
                   _lowState->motorState[0].q, _lowState->motorState[1].q, _lowState->motorState[2].q);
            printf("[调试] 当前阶段：t=%.2fs | 增益 Kp=%.1f Kd=%.2f tau=%.2f\n", t_cycle, g.kp, g.kd, g.tau);
            std::cout << "------------------------------------\n";
        }
    }

    // IMU姿态输出
    static int imu_cnt = 0;
    if (++imu_cnt % 30 == 0) {
        printf("[IMU] 四元数：w=%.3f x=%.3f y=%.3f z=%.3f\n",
               _lowState->imu.quaternion[0], _lowState->imu.quaternion[1],
               _lowState->imu.quaternion[2], _lowState->imu.quaternion[3]);
    }
}

/* -------------------- exit -------------------- */
void State_StepTest::exit() {
    std::cout << "\n[JumpTest] 退出：平滑回初始位置...\n";
    
    // 逐步降低增益，避免电机突然失能
    for (int leg = 0; leg < 4; ++leg) {
        int leg_base_id = leg * 3;
        Vec3 init_q(HIP_JOINT_FIXED, _initMotorQ[leg_base_id + 1], _initMotorQ[leg_base_id + 2]);
        for (int j = 0; j < 3; ++j) {
            int id = leg_base_id + j;
            _lowCmd->motorCmd[id].q = init_q(j);
            _lowCmd->motorCmd[id].Kp = 5.0f;  // 逐渐降低Kp
            _lowCmd->motorCmd[id].Kd = 0.15f;
            _lowCmd->motorCmd[id].tau = 0.0f;
            _lowCmd->motorCmd[id].mode = 1; // 确保模式正确
        }
    }
    
    // 等待短暂时间让电机稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 最后切换到固定站立状态
    for (int i = 0; i < 12; ++i) {
        _lowCmd->motorCmd[i].q = _initMotorQ[i];
        _lowCmd->motorCmd[i].Kp = 5.0f;
        _lowCmd->motorCmd[i].Kd = 0.1f;
        _lowCmd->motorCmd[i].tau = 0.0f;
        _lowCmd->motorCmd[i].mode = 1;
    }
    
    for (int leg = 0; leg < 4; ++leg) {
        _lowCmd->motorCmd[leg * 3].q = HIP_JOINT_FIXED;
    }
    
    std::cout << "[JumpTest] 退出完成！已回Trotting\n";
    std::cout << "[INFO] 轨迹数据存至：" << CSV_PATH << "\n";
}

/* -------------------- checkChange -------------------- */
FSMStateName State_StepTest::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B) return FSMStateName::PASSIVE;
    if (_lowState->userCmd == UserCommand::L2_A) return FSMStateName::FIXEDSTAND;
    if (_lowState->userCmd == UserCommand::L2_X) return FSMStateName::TROTTING;
    if (_isJumpCompleted) return FSMStateName::TROTTING;
    return FSMStateName::STEPTEST;
}

void State_StepTest::calcTau() { }



