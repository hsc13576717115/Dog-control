#include "FSM/State_FixedStand.h"
#include "FSM/State_SwingTest.h"  
#include "interface/IOSDK.h"      
#include <iostream>
#include <iomanip>              
#include <Eigen/Core>
#include <cmath>                 

static constexpr double L0 = 0.08415;   // 髋偏移（髋轴→腿平面）
static constexpr double L1 = 0.213;   // 大腿长度
static constexpr double L2 = 0.213;   // 小腿长度 
static constexpr double Q0_LIMIT_MIN = -2.60;  
static constexpr double Q0_LIMIT_MAX =  2.60;  
static constexpr double Q1_LIMIT_MIN = -6.50; 
static constexpr double Q1_LIMIT_MAX = 6.50;   
static constexpr double Q2_LIMIT_MIN = -2.30; 
static constexpr double Q2_LIMIT_MAX = 2.30;  

// 构造函数：初始化基础成员，不硬编码目标角度（目标角度在enter中通过IK计算）
State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"),  // 基类初始化
      _pHip2B(0.1525f, -0.0565f, 0.0f),  // 髋部偏移（与SwingTest/IOSDK严格统一）
      _debugMotorEnable(false),           // 默认关闭调试模式
      _debugMotorID(8),                   // 默认调试8号电机（可根据需求修改）
      _duration(500),                     // 500个控制周期过渡（约5秒，避免抽搐）
      _percent(0.0f),                     // 初始进度0
      _currentMode(NORMAL_STAND),         // 初始为正常站立模式
      _lastMode(NORMAL_STAND)             // 上次模式
{
    _targetPos.setZero();  // 目标角度初始化为全零，后续通过IK更新
}

// 进入状态：核心逻辑→根据目标足端坐标，通过IK反算关节角度
void State_FixedStand::enter()
{
    std::cout << "[DEBUG] 进入 FixedStand 状态" << std::endl;

    /* 1. 增益初始化 */
    for (int i = 0; i < 4; ++i) {
        if (_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO)
            _lowCmd->setSimStanceGain(i);
        else
            _lowCmd->setRealStanceGain(i);
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }

    /* 2. 记录初始角度 */
    for (int i = 0; i < 12; ++i) {
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i]           = _lowState->motorState[i].q;
    }

    /* 3. 校准检查 */
    IOSDK* iosdk = dynamic_cast<IOSDK*>(_ctrlComp->ioInter);
    if (!iosdk || !iosdk->isCalibrated()) {
        std::cerr << "[FixedStand] 警告：未完成 L 型校准！先按 L1+X 校准！" << std::endl;
        _targetPos.setZero();
        _ctrlComp->setAllStance();
        _percent = 0.0f;
        return;
    }

    /* 4. 根据当前模式选择目标姿态 */
    Vec3 target[4];
    if (_currentMode == NORMAL_STAND) {
        // 正常站立姿态
        target[0] = Vec3(-0.01,  0.08415, -0.22);   // leg0  FL
        target[1] = Vec3(-0.01, -0.08415, -0.22);   // leg1  FR
        target[2] = Vec3(-0.01,  0.08415, -0.22);   // leg2  RL
        target[3] = Vec3(-0.01, -0.08415, -0.22);   // leg3  RR
        std::cout << "[FixedStand] 当前模式 - 正常站立姿态" << std::endl;
    } else {
        // 匍匐姿态
        target[0] = Vec3(-0.01,  0.18, -0.15);   // leg0  FL
        target[1] = Vec3(-0.01, -0.18, -0.15);   // leg1  FR
        target[2] = Vec3(-0.01,  0.18, -0.15);   // leg2  RL
        target[3] = Vec3(-0.01, -0.18, -0.15);   // leg3  RR
        std::cout << "[FixedStand] 当前模式 - 匍匐姿态" << std::endl;
    }

    /* 5. 可达性检查 + 限幅 */
    Vec3 tgt[4];
    for (int leg = 0; leg < 4; ++leg) {
        tgt[leg] = target[leg];
        double dist = tgt[leg].norm();
        double dMin = std::fabs(L1 - L2);
        double dMax = L1 + L2;
        if (dist < dMin - 1e-3 || dist > dMax + 1e-3) {
            std::cerr << "[FixedStand] leg" << leg
                      << " 不可达 dist=" << dist << std::endl;
            tgt[leg] = tgt[leg].normalized() * dMax * 0.9;
        }
    }

    /* 6. 逐腿 IK → 加偏移 → 限位 → 减偏移 */
    Vec12 q12;
    for (int leg = 0; leg < 4; ++leg) {
        Vec3 qRel = ikCheckAxis(tgt[leg], leg);

        Vec3 off;
        for (int m = 0; m < 3; ++m) off(m) = iosdk->getCalibOffset()[leg * 3 + m];

        Vec3 qRaw = qRel + off;
        qRaw(0) = clamp(qRaw(0), Q0_LIMIT_MIN, Q0_LIMIT_MAX);
        qRaw(1) = clamp(qRaw(1), Q1_LIMIT_MIN, Q1_LIMIT_MAX);
        qRaw(2) = clamp(qRaw(2), Q2_LIMIT_MIN, Q2_LIMIT_MAX);
        Vec3 qCmd = qRaw - off;

        int b = leg * 3;
        q12(b + 0) = qCmd(0);
        q12(b + 1) = qCmd(1);
        q12(b + 2) = qCmd(2);
    }
    _targetPos = q12;

    /* 7. FK 验证（leg0） */
    Vec3 footCheck = fkCheckAxis(ikCheckAxis(tgt[0], 0));
    double err = (tgt[0] - footCheck).norm() * 1000.0;
    std::cout << "[FixedStand] FK 回算误差: " << std::fixed << std::setprecision(2)
              << err << " mm" << std::endl;

    /* 8. 状态初始化 */
    _ctrlComp->setAllStance();
    _percent = 0.0f;
}

void State_FixedStand::run() {
    // 更新过渡进度（每周期递增1/_duration，最大1.0）
    _percent += 1.0f / _duration;
    if (_percent > 1.0f) {
        _percent = 1.0f;  // 进度上限100%
    }

    // 模式1：单电机调试（用于单独测试某个电机，默认关闭）
    if (_debugMotorEnable) {
        int debug_id = _debugMotorID;
        // 平滑过渡公式：初始角度 + 进度*(目标角度-初始角度)
        _lowCmd->motorCmd[debug_id].q = (1.0f - _percent) * _startPos[debug_id] + _percent * _targetPos[debug_id];
        _lowCmd->motorCmd[debug_id].mode = 1;  // 位置控制模式

        // 其他电机保持关闭（避免干扰调试）
        for (int k = 0; k < 12; k++) {
            if (k != debug_id) {
                _lowCmd->motorCmd[k].mode = 0;  // 关闭电机
                _lowCmd->motorCmd[k].q = 0.0f;
            }
        }

        // 打印调试信息（每周期输出）
        std::cout << "[DEBUG] 单电机调试 | ID=" << debug_id 
                  << " | 目标角度=" << std::fixed << std::setprecision(4) << _targetPos[debug_id] << " rad"
                  << " | 当前角度=" << std::fixed << std::setprecision(4) << _lowCmd->motorCmd[debug_id].q << " rad"
                  << " | 进度=" << std::fixed << std::setprecision(1) << _percent * 100 << "%" << std::endl;
    } 
    // 模式2：全局站立（控制所有12轴电机，默认模式）
    else {
        // 所有电机平滑过渡到目标角度
        for (int j = 0; j < 12; j++) {
            //调试xbot时注释
            _lowCmd->motorCmd[j].mode = 1;  // 位置控制模式
            _lowCmd->motorCmd[j].q = (1.0f - _percent) * _startPos[j] + _percent * _targetPos[j];
        }

        // 每100ms打印一次站立状态（验证足端坐标是否达标）
        static int print_cnt = 0;
        if (++print_cnt % 10 == 0) {
            // 取0号腿的实时角度，FK回算当前足端坐标
            Vec3 q_current(0.0, _lowState->motorState[1].q, _lowState->motorState[2].q);
            Vec3 foot_current = fkCheckAxis(q_current);
            // 目标足端坐标（与enter中一致）
            Vec3 target_foot_axis(-0.05,  0.10, -0.28);   // 向后 22.5 cm，向下 35 cm
            // 计算当前位置误差（mm）
            double current_error = sqrt(pow(target_foot_axis.x() - foot_current.x(), 2) + 
                                       pow(target_foot_axis.z() - foot_current.z(), 2)) * 1000;

            // 格式化打印状态
            // std::cout << "\n================================ FixedStand 状态 ================================" << std::endl;
            // std::cout << "过渡进度：" << std::fixed << std::setprecision(1) << _percent * 100 << "%" << std::endl;
            // std::cout << "目标足端（轴心系）：x=" << std::fixed << std::setprecision(4) << target_foot_axis.x()
            //           << " m, z=" << std::fixed << std::setprecision(4) << target_foot_axis.z() << " m" << std::endl;
            // std::cout << "当前足端（轴心系）：x=" << std::fixed << std::setprecision(4) << foot_current.x() 
            //           << " m, z=" << std::fixed << std::setprecision(4) << foot_current.z() << " m" << std::endl;
            // std::cout << "当前坐标误差：" << std::fixed << std::setprecision(2) << current_error << " mm" << std::endl;
            // std::cout << "===============================================================================" << std::endl;
        }
                // 🔍 调试：打印下发 vs 回读（50 周期一次）
        // static int db_cnt = 0;
        // if (++db_cnt % 50 == 0) {
        //     printf("[DEBUG] 下发 大腿=%+7.3f 小腿=%+7.3f   回读 大腿=%+7.3f 小腿=%+7.3f\n",
        //            _targetPos[1], _targetPos[2],
        //            _lowState->motorState[1].q, _lowState->motorState[2].q);
        // }
    }
}

// 退出状态：重置过渡进度，切换到下一个模式
void State_FixedStand::exit() {
    _percent = 0.0f;
    
    // 切换模式：在正常站立和匍匐之间切换
    if (_currentMode == NORMAL_STAND) {
        _currentMode = CROUCH;
    } else {
        _currentMode = NORMAL_STAND;
    }
    
    std::cout << "[FixedStand] 退出站立状态，已重置过渡进度，下次进入将使用" 
              << (_currentMode == NORMAL_STAND ? "正常站立" : "匍匐") << "模式" << std::endl;
}

// 状态切换判断：根据用户指令切换到其他状态
FSMStateName State_FixedStand::checkChange() {
    if (_lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;       // L2+B → 被动模式（电机断电）
    } else if (_lowState->userCmd == UserCommand::L2_X) {
    //     return FSMStateName::FREESTAND;     // L2+X → 自由站立模式
    // } else if (_lowState->userCmd == UserCommand::START) {
        return FSMStateName::TROTTING;      // START键 → Trot步态模式
    } else if (_lowState->userCmd == UserCommand::L1_X) {
    //     return FSMStateName::BALANCETEST;   // L1+X → 平衡测试模式
    // } else if (_lowState->userCmd == UserCommand::L1_A) {
    //     return FSMStateName::SWINGTEST;     // L1+A → 摆动测试模式（基准一致，切换平滑）
    // } else if (_lowState->userCmd == UserCommand::L1_Y) {
    //     return FSMStateName::STEPTEST;      // L1+Y → 步长测试模式
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if (_lowState->userCmd == UserCommand::L2_Y) {
        return FSMStateName::MOVE_BASE;     // L2+Y → 移动基座模式（若启用ROS）
    }
#endif
    return FSMStateName::FIXEDSTAND;
}



