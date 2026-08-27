//
// Created by yaozhuo on 8/26/26.
//

#ifndef UMI_ON_AIR_ARM_ARM_3DOF_H
#define UMI_ON_AIR_ARM_ARM_3DOF_H

#pragma once
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>
#include <iostream>
#include <vector>
#include <cmath>

/**
 * UMI‑on‑Air scorpion arm: ONLY 3 pitch joints θ1,θ2,θ3
 * remove wrist roll joint4.
 * q = [th1, th2, th3]
 * All joints rotate around local +X (pitch).
 * End‑effector lies on y=0 plane in arm‑base frame {B}.
 */

// TODO: get accurate length and
// struct Scorpion3Param
// {
//     double l1{0.38}; // 0.37977 = sqrt(0.375^2 + 0.06^2)
//     double l2{0.41};
//     double l3{0.11};

//     // 关节限位 [th1, th2, th3]，默认俯仰角度限制(rad)
//     Eigen::Vector3d q_min{0, -M_PI/2, -M_PI/2};
//     Eigen::Vector3d q_max{ M_PI,  M_PI/2,  M_PI/2};

// };

struct Scorpion3Param
{
    double l0{0.12};
    double l1{0.18};
    double l2{0.22};
    double l3{0.16};
    double lg{0.08};
    inline double L3g() const noexcept { return l3 + lg; }
};

// 求解返回状态
enum class IK3Status
{
    OK,                 ///< 存在满足关节限位的解
    NO_GEOM_SOLUTION,   ///< 几何工作空间内不存在解
    NO_VALID_SOLUTION   ///< 几何有解，但全部候选解违反关节限位
};

struct IK3Result
{
    std::vector<Eigen::Vector3d> candidates;
    IK3Status status;
};

IK3Result scorpion3_ik(double xd, double zd, double psid, const Scorpion3Param& param);

Eigen::Vector3d select_nearest_solution(const std::vector<Eigen::Vector3d>& candidates,
                                         const Eigen::Vector3d& q_curr);


// 配套正运动学（本构型原版）
struct Scorpion3FKRes
{
    double x;
    double z;
    double psi;
    Eigen::Vector3d pos() const { return {x,0.0,z}; }
};

Scorpion3FKRes scorpion3_fk(const Eigen::Vector3d& q, const Scorpion3Param& param);
           

#endif //UMI_ON_AIR_ARM_ARM_3DOF_H
