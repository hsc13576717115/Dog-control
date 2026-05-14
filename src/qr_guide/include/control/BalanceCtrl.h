/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
 移植自 unitree_guide：基于 QP 的足端力分配器。
 给定期望的机身质心加速度，通过二次规划求解各支撑足的地面反力，
 满足：
   (1) 质心动量方程约束；
   (2) 摆动腿零力等式约束；
   (3) 摩擦锥与单向承压不等式约束。
***********************************************************************/
#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include "common/mathTypes.h"
#include "config/RobotConfig.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "common/unitreeRobot.h"

/**
 * @brief 力平衡 QP 控制器
 *
 * 核心公式：
 *   min   (A*f - b_d)^T * S * (A*f - b_d) + alpha * f^T * W * f + beta * (f - f_prev)^T * U * (f - f_prev)
 *   s.t.  f_swing = 0                       (等式约束)
 *         |f_x|,|f_y| <= mu * f_z, f_z >= 0  (不等式约束，摩擦锥)
 *
 * 其中 A 为 6x12 映射矩阵，将足端力映射到质心广义力（线加速度+角加速度）。
 */
class BalanceCtrl {
public:
    /**
     * @brief 显式参数构造
     */
    BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta);

    /**
     * @brief 从 QuadrupedRobot 模型自动提取参数（默认权重）
     */
    BalanceCtrl(QuadrupedRobot* robModel);

    /**
     * @brief 从 QuadrupedRobot 模型和 YAML 参数构造
     */
    BalanceCtrl(QuadrupedRobot* robModel, const qr_guide::ForceControlParameters& params);

    /**
     * @brief 计算足端力
     * @param ddPcd   期望机身线加速度 (world frame)
     * @param dWbd    期望机身角加速度 (world frame)
     * @param rotM    机身旋转矩阵 (body -> world)
     * @param feetPos2B  足端相对于机身质心的位置 (world frame)
     * @param contact    接触状态 (1=支撑, 0=摆动)
     * @return Vec34  12维足端力向量，按 FR/FL/RR/RL 每列3维排列
     */
    Vec34 calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact);

private:
    void calMatrixA(Vec34 feetPos2B, RotMat rotM, VecInt4 contact);
    void calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM);
    void calConstraints(VecInt4 contact);
    void solveQP();

    // QP 代价矩阵
    Mat12 _G, _W, _U;
    Mat6 _S;           // 质心动力学跟踪权重
    Mat3 _Ib;          // 机身惯性张量 (body frame)
    Vec6 _bd;          // 期望质心广义力 [m*(ddP-g); I_global*dWb]
    Vec3 _g;           // 重力加速度
    Vec3 _pcb;         // CoM 偏移
    Vec12 _F;          // 当前解 (12维)
    Vec12 _Fprev;      // 上一时刻解（平滑项用）
    Vec12 _g0T;        // 线性项系数
    double _mass;      // 总质量
    double _alpha;     // 力正则化权重
    double _beta;      // 时间平滑权重
    double _fricRatio; // 摩擦系数

    // 约束矩阵（动态大小，因为接触腿数变化）
    Eigen::MatrixXd _CE, _CI;
    Eigen::VectorXd _ce0, _ci0;

    // 映射矩阵 A: 6x12
    Eigen::Matrix<double, 6, 12> _A;

    // 摩擦锥矩阵: 5x3
    // [ 1,  0, mu]
    // [-1,  0, mu]
    // [ 0,  1, mu]
    // [ 0, -1, mu]
    // [ 0,  0,  1]
    Eigen::Matrix<double, 5, 3> _fricMat;

    // quadProgpp 工作区（动态大小，每轮重新分配）
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;
};

#endif  // BALANCECTRL_H
