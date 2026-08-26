//
// Created by yaozhuo on 8/26/26.
//

#ifndef UMI_ON_AIR_ARM_ARM_3DOF_H
#define UMI_ON_AIR_ARM_ARM_3DOF_H

#pragma once
#include <Eigen/Dense>
#include <Eigen/SVD>

/**
 * UMI‑on‑Air scorpion arm: ONLY 3 pitch joints θ1,θ2,θ3
 * remove wrist roll joint4.
 * q = [th1, th2, th3]
 * All joints rotate around local +X (pitch).
 * End‑effector lies on y=0 plane in arm‑base frame {B}.
 */

// TODO: get accurate length and
struct Scorpion3Param
{
    double l1{0.38}; // 0.37977 = sqrt(0.375^2 + 0.06^2)
    double l2{0.41};
    double l3{0.11};

    // 关节限位 [th1, th2, th3]，默认俯仰角度限制(rad)
    Eigen::Vector3d q_min{0, -M_PI/2, -M_PI/2};
    Eigen::Vector3d q_max{ M_PI,  M_PI/2,  M_PI/2};

};

/**
 * @brief forward kinematics, arm‑base frame {B}
 * @param q [th1, th2, th3] rad
 * @return p_ee (x, 0, z)
 */
Eigen::Vector3d scorpion3_fk(const Eigen::Vector3d& q, const Scorpion3Param& param = Scorpion3Param{});

/**
 * @brief 3‑DOF蝎子臂，计算全部关节空间位置
 * @param q [th1, th2, th3] 三个俯仰关节角(rad)
 * @param param 连杆参数
 * @return std::vector<Eigen::Vector3d> 顺序：base, j1, j2, j3, end‑effector
 *         y分量恒为0，全部在X‑Z平面
 */
std::vector<Eigen::Vector3d> scorpion3_fk_all_joints(const Eigen::Vector3d& q, const Scorpion3Param& param = Scorpion3Param{});

/**
 * @brief geometric jacobian J(3×3): dp / dq
 */
Eigen::Matrix<double,3,3> scorpion3_jacobian(const Eigen::Vector3d& q, const Scorpion3Param& param = Scorpion3Param{});

/**
 * @brief 冗余IK：位置任务 + 零空间优化，寻找距离q_init最近的关节解
 * @param p_des 臂基坐标系目标，y=0
 * @param q_init 初始/参考关节角度（要靠近这个）
 * @param task_gain 位置跟踪增益
 * @param null_gain 零空间回归增益(向q_init拉)
 */
Eigen::Vector3d scorpion3_ik_nullspace(const Eigen::Vector3d& p_des,
                                       const Eigen::Vector3d& q_init,
                                       int max_iter = 1e4,
                                       double tol = 1e-3,
                                       double task_gain = 1.0,
                                       double null_gain = 0.2,
                                       const Scorpion3Param& param = Scorpion3Param{});

/**
 * @brief 快速预检查：末端目标点是否几何可达（不考虑关节角度限位）
 * @param xd 目标末端 [x, z]
 * @param param 臂连杆参数
 * @param eps 数值容差，处理浮点误差
 * @return true：点在工作空间圆盘/圆环内，可进入IK求解；false：完全不可达
 */
bool scorpion3_check_reachable(double x, double z,
                               const Scorpion3Param& param = Scorpion3Param{},
                               double eps = 1e-8);

#endif //UMI_ON_AIR_ARM_ARM_3DOF_H
