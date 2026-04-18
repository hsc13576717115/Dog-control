#ifndef STATE_FIXEDSTAND_H
#define STATE_FIXEDSTAND_H

#include <array>
#include <memory>

#include "control/HybridStandController.h"
#include "FSMState.h"

enum StandMode {
    NORMAL_STAND = 0,
    CROUCH = 1,
};

// 固定站立状态。
// 第一阶段采用纯位置控制：
// 足端空间插值 -> 小幅姿态/高度补偿 -> IK -> 关节位置命令。
class State_FixedStand : public FSMState {
public:
    explicit State_FixedStand(CtrlComponents* ctrlComp);
    ~State_FixedStand() override = default;

    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    Vec3 clampJointAngles(const Vec3& q) const;
    float transitionBlend() const;
    Vec3 computeBaseStandFootTargetInHip(int leg) const;
    Vec3 computeCompensatedFootTargetInHip(int leg, const Vec3& base_target_in_hip) const;

    int _duration = 500;
    float _percent = 0.0f;
    int _debugPrintCounter = 0;
    std::array<Vec3, qr_guide::NumLeg> _startFeetInHip{};
    std::array<Vec3, qr_guide::NumLeg> _targetFeetInHip{};
    std::array<Vec3, qr_guide::NumLeg> _targetFeetInBody{};
    std::unique_ptr<qr_guide::HybridStandController> _hybridStandController;
    StandMode _currentMode = NORMAL_STAND;
};

#endif  // STATE_FIXEDSTAND_H
