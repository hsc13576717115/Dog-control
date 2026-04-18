#ifndef IOSDK_H
#define IOSDK_H

#include <array>
#include <condition_variable>
#include <cmath>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "config/RobotConfig.h"
#include "interface/IOInterface.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

// 真实电机 IO 层。
// 当前职责只保留：串口通信、减速比换算、校准偏差和反馈回填。
class IOSDK : public IOInterface {
public:
    explicit IOSDK(const qr_guide::DriveParameters& drive_parameters);
    ~IOSDK() override;

    void sendRecv(const UserLowlevel::LowlevelCmd* cmd, LowlevelState* state) override;
    bool isCalibrated() const override { return _isCalibrated; }
    const std::array<float, 12>& getCalibOffset() const { return _calibOffset; }

private:
    void openSerialPorts();
    void initializeMotorMetadata();
    void runStartupPoseAlignment();
    void startWorkers();
    void stopWorkers();
    void workerLoop(int leg);
    void maybePrintCalibrationReminder();
    void tryCalibrate(const LowlevelState& state);
    void calibrateLeg(int leg);
    void refreshMotorFeedback();
    void sendDirectLegCommand(int leg, const std::array<UserLowlevel::MotorCmd, 3>& user_cmds);
    std::array<UserLowlevel::MotorCmd, 3> buildAlignmentCommand(int leg, bool enable_calf) const;
    std::array<UserLowlevel::MotorCmd, 3> buildZeroTorqueCommand() const;
    void sendReceiveLeg(int leg, const UserLowlevel::LowlevelCmd* cmd, LowlevelState* state);
    void populateMotorCommand(int leg, int joint, const UserLowlevel::MotorCmd& user_cmd);
    void updateMotorStateFromFeedback(int leg, int joint, LowlevelState* state);
    void markMotorOffline(int leg, int joint, LowlevelState* state) const;
    float gearRatioForJoint(int joint) const;
    float calibrationTargetUserAngle(int leg, int joint) const;

    std::vector<SerialPort*> _serials;
    // 串口顺序固定为 FR / FL / RR / RL。
    std::array<std::string, 4> _serialPorts;
    MotorCmd _motorCmd[12];
    MotorData _motorData[12];
    std::set<int> _activeLegs;
    std::array<float, 12> _calibOffset;
    std::array<std::thread, 4> _workers;
    std::mutex _workerMutex;
    std::condition_variable _dispatchCv;
    std::condition_variable _completedCv;
    const UserLowlevel::LowlevelCmd* _activeCmd = nullptr;
    LowlevelState* _activeState = nullptr;
    std::size_t _dispatchEpoch = 0;
    std::size_t _completedWorkers = 0;
    bool _workersStopping = false;
    bool _isCalibrated = false;
    bool _startupAlignmentDone = false;
    bool _useParallelLegIo = false;
    // 校准仍然沿用 START -> L1_X 这条触发链路。
    const UserCommand _calibTriggerKey = UserCommand::L1_X;
    const double _calibPromptIntervalSec = 5.0;
    double _lastCalibPromptTimeSec = 0.0;
    static constexpr float _degToRad = static_cast<float>(M_PI) / 180.0f;
    float _baseGearRatio = 6.33f;
    float _calfTotalGearRatio = 12.66f;
    float _calibrationHipAngleRad = 0.0f * _degToRad;
    float _calibrationThighAngleRad = -161.8f * _degToRad;
    float _calibrationCalfAngleRad = -71.8f * _degToRad;
    float _startupHipTauNm = 0.15f;
    float _startupThighTauNm = 0.10f;
    float _startupCalfTauNm = 0.20f;
    int _startupAlignmentPhase1Ms = 300;
    int _startupAlignmentPhase2Ms = 300;
    int _startupAlignmentReleaseMs = 300;
};

#endif  // IOSDK_H
