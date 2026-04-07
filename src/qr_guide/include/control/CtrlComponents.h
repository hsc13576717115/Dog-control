#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include <memory>

#include "config/RobotConfig.h"
#include "control/Estimator.h"
#include "interface/IOInterface.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"

class ControllerContext {
public:
    // 上下文负责持有整条控制链共用的运行时对象。
    ControllerContext(std::unique_ptr<IOInterface> io_inter,
                      std::unique_ptr<QuadrupedRobot> robot_model,
                      const qr_guide::RobotParameters& parameters)
        : lowCmd(std::make_unique<UserLowlevel::LowlevelCmd>()),
          lowState(std::make_unique<LowlevelState>()),
          ioInter(std::move(io_inter)),
          robotModel(std::move(robot_model)),
          estimator(nullptr),
          parameters(parameters) {
        setAllSwing();
    }

    void initialize() {
        // 估计器依赖 robotModel、lowState 和 contact/phase，因此集中在这里初始化。
        estimator = std::make_unique<Estimator>(robotModel.get(), lowState.get(), &contact, &phase, dt);
    }

    void sendRecv() {
        ioInter->sendRecv(lowCmd.get(), lowState.get());
    }

    void setAllStance() {
        contact = VecInt4::Ones();
        phase = Vec4::Constant(0.5);
    }

    void setAllSwing() {
        contact = VecInt4::Zero();
        phase = Vec4::Constant(0.5);
    }

    void setStartWave() {
        phase << 0.0, 0.5, 0.5, 0.0;
        contact << 1, 0, 0, 1;
    }

    void setContactPhase(const VecInt4& new_contact, const Vec4& new_phase) {
        contact = new_contact;
        phase = new_phase;
    }

    bool isCalibrated() const { return ioInter && ioInter->isCalibrated(); }

    std::unique_ptr<UserLowlevel::LowlevelCmd> lowCmd;
    std::unique_ptr<LowlevelState> lowState;
    std::unique_ptr<IOInterface> ioInter;
    std::unique_ptr<QuadrupedRobot> robotModel;
    std::unique_ptr<Estimator> estimator;
    qr_guide::RobotParameters parameters;
    VecInt4 contact = VecInt4::Zero();
    Vec4 phase = Vec4::Constant(0.5);
    double dt = 0.002;
    CtrlPlatform ctrlPlatform = CtrlPlatform::REALROBOT;
};

using CtrlComponents = ControllerContext;

#endif  // CTRLCOMPONENTS_H
