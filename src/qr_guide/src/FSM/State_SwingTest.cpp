#include "FSM/State_SwingTest.h"
#include "interface/IOSDK.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

/* =========================================================
 * 1. 机械与关节参数（轴心坐标系）
 * ========================================================= */
static constexpr double L0 = 0.08415;   // 髋偏移（髋轴→腿平面）
static constexpr double L1 = 0.213;   // 大腿长度
static constexpr double L2 = 0.213;   // 小腿长度

static constexpr double Q0_LIMIT_MIN = -2.60;
static constexpr double Q0_LIMIT_MAX =  2.60;
static constexpr double Q1_LIMIT_MIN = -6.50;
static constexpr double Q1_LIMIT_MAX =  6.50;
static constexpr double Q2_LIMIT_MIN = -2.30;
static constexpr double Q2_LIMIT_MAX =  2.30;

/* =========================================================
 * 2. FK（轴心坐标系）
 * ========================================================= */
Vec3 fkCheckAxis(const Vec3 &qUser, int leg)
{
    /* -------- leg 属性 -------- */
    const bool isFront = (leg == 0 || leg == 1);
    const bool isLeft  = (leg == 1 || leg == 3);

    /* -------- 输入角 -------- */
    double q0 = qUser(0);   // roll
    double q1 = qUser(1);   // thigh (relative)
    double q2 = qUser(2);   // calf  (relative)

    /* -------- 左右腿镜像（必须与 IK 完全一致） -------- */
    if (isFront) q0 = -q0;
    if (isLeft)  q1 = -q1, q2 = -q2;

    /* -------- 绝对角 -------- */
    const double q1_abs = q1;
    const double q2_abs = M_PI_2 - q2;
    const double knee   = q1_abs + q2_abs;

    /* -------- 二连杆平面 FK -------- */
    const double x2 = L1 * std::cos(q1_abs) + L2 * std::cos(knee);
    const double z2 = L1 * std::sin(q1_abs) + L2 * std::sin(knee);

    /* -------- 髋偏移 -------- */
    const double y_off = isLeft ? -L0 : L0;

    /* -------- 绕 X 的 roll -------- */
    const double xw = x2;
    const double yw = y_off * std::cos(q0) - z2 * std::sin(q0);
    const double zw = y_off * std::sin(q0) + z2 * std::cos(q0);

    return Vec3(xw, yw, zw);
}

/* =========================================================
 * 3. IK（轴心坐标系）
 * ========================================================= */
Vec3 ikCheckAxis(const Vec3 &pDes, int leg)
{
    const bool isFront = (leg == 0 || leg == 1);
    const bool isLeft  = (leg == 1 || leg == 3);

    const double x = pDes.x();
    const double y = pDes.y();
    const double z = pDes.z();

    const double y_off = isLeft ? -L0 : L0;

    /* -------- q0 求解 -------- */
    double q0 = 0.0;
    const double A = y;
    const double B = z;
    const double denom = A*A + B*B;

    if (denom > 1e-6) {
        const double root = std::sqrt(std::max(0.0, denom - y_off*y_off));
        const double sin_q0 = (B*y_off + A*root) / denom;
        const double cos_q0 = (y_off - B*sin_q0) / A;
        q0 = std::atan2(sin_q0, cos_q0);
    }

    /* -------- 回到 2D 平面 -------- */
    const double x2 = -x;
    const double z2 = z * std::cos(q0) - y * std::sin(q0);

    /* -------- 平面二连杆 IK -------- */
    const double r = std::hypot(x2, z2);
    const double r_min = std::fabs(L1 - L2);
    const double r_max = L1 + L2;

    static Vec3 qLast(0.0, 0.3, -0.8);
    if (r < r_min || r > r_max) {
        std::cerr << "[IK] 不可达 r=" << r << "\n";
        return qLast;
    }

    const double alpha = std::atan2(-z2, -x2);
    const double beta  = std::acos(clamp((L1*L1 + r*r - L2*L2)/(2*L1*r), -1.0, 1.0));
    const double c2    = clamp((L1*L1 + L2*L2 - r*r)/(2*L1*L2), -1.0, 1.0);

    Vec3 qBack (q0, -(alpha + beta), -M_PI_2 +  std::acos(c2));
    Vec3 qFront(q0, -(alpha - beta), -M_PI_2 -  std::acos(c2));

    auto inside = [](const Vec3 &q){
        return q(1)>=Q1_LIMIT_MIN && q(1)<=Q1_LIMIT_MAX &&
               q(2)>=Q2_LIMIT_MIN && q(2)<=Q2_LIMIT_MAX;
    };

    Vec3 q = inside(qBack) ? qBack : qFront;

    /* -------- 镜像回用户定义 -------- */
    if (isFront) q(0) = -q(0);
    if (isLeft)  q(1) = -q(1), q(2) = -q(2);

    qLast = q;
    return q;
}

/* =========================================================
 * 4. 调试打印
 * ========================================================= */
static void printXZ(const std::string &tag, const Vec3 &v)
{
    std::cout << "[SwingTest] " << tag
              << " x=" << std::fixed << std::setprecision(4) << v.x()
              << " z=" << std::fixed << std::setprecision(4) << v.z()
              << " m\n";
}

/* =========================================================
 * 5. FSM：构造
 * ========================================================= */
State_SwingTest::State_SwingTest(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::SWINGTEST, "swingTest"),
      _pHip2B(0.1525, -0.0565, 0.0),
      _xRange(0.020),
      _zRange(0.020),
      _firstRun(true),
      _transPct(0.0f),
      _standQ(Vec3::Zero()),
      _enterQ(Vec3::Zero()),
      _prevQ(Vec3::Zero()),
      _standXZ(Vec3::Zero()),
      _goalXZ(Vec3::Zero()),
      _smoothXZ(Vec3::Zero())
{}

/* =========================================================
 * 6. enter
 * ========================================================= */
void State_SwingTest::enter()
{
    for (int i = 0; i < 4; ++i) {
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }

    _prevQ  = Vec3(0.0, _lowState->motorState[1].q,
                          _lowState->motorState[2].q);
    _enterQ = _prevQ;
    _standQ = _prevQ;

    _standXZ  = fkCheckAxis(_standQ, 0);
    _goalXZ   = _standXZ;
    _smoothXZ = _standXZ;

    _transPct = 0.0f;
    _firstRun = true;

    printXZ("Enter Foot", _standXZ);
    _ctrlComp->setAllSwing();
}

/* =========================================================
 * 7. run
 * ========================================================= */
void State_SwingTest::run()
{
    _userValue = _lowState->userValue;

    const double dx = _userValue.ly * _xRange;
    const double dz = _userValue.ry * _zRange;

    _goalXZ.x() = _standXZ.x() + dx;
    _goalXZ.z() = _standXZ.z() + dz;

    _smoothXZ = 0.95 * _smoothXZ + 0.05 * _goalXZ;

    Vec3 qDes = ikCheckAxis(_smoothXZ, 0);

    if (_transPct < 1.0f) {
        _transPct += 1.0f / _transitionDuration;
        qDes = _prevQ + (_enterQ - _prevQ) * _transPct;
    }

    Vec12 qCmd;
    for (int i = 0; i < 12; ++i)
        qCmd(i) = _lowState->motorState[i].q;

    qCmd(1) = qDes(1);
    qCmd(2) = qDes(2);

    _lowCmd->setQ(qCmd);
}

/* =========================================================
 * 8. exit / checkChange
 * ========================================================= */
void State_SwingTest::exit()
{
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_SwingTest::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)
        return FSMStateName::PASSIVE;
    if (_lowState->userCmd == UserCommand::L2_A)
        return FSMStateName::FIXEDSTAND;
    return FSMStateName::SWINGTEST;
}

/* =========================================================
 * 9. 旧接口兼容
 * ========================================================= */
Vec3 fkCheck(const Vec3 &qUser, const Vec3 &hip2B)
{
    return fkCheckAxis(qUser, 0) + hip2B;
}
