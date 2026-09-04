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
// 机械臂参数：3连杆，3个绕Y俯仰旋转关节

struct Scorpion3Param
{
    double l1{0.38};
    double l2{0.41};
    double l3{0.11};

    // 每个关节独立限位：th1, th2, th3
    Eigen::Vector3d q_min{-0.8477, -1.7683, -1.9218};
    Eigen::Vector3d q_max{ 1.3004,  1.1469,  1.6072};

    double q4_min = -1.4615f, q4_max = 1.6072f; //机械臂最后一个扭转电机关节限位
};

// 正运动学：全部关节点位置输出
struct Scorpion3AllJointsPos
{
    Eigen::Vector3d J1;  // 关节1 肩关节
    Eigen::Vector3d J2;  // 关节2 肘关节
    Eigen::Vector3d J3;  // 关节3 腕关节
    Eigen::Vector3d EE;  // 第三连杆末端执行点
    double psi;          // 末端绕Y俯仰角 psi = th1+th2+th3
};

// 简易单末端FK，只返回末端x,z,psi
struct Scorpion3FKRes
{
    double x;
    double z;
    double psi;
    Eigen::Vector3d pos() const { return {x, 0.0, z}; }
};

// 解析逆解状态
enum class IK3Status
{
    OK = 0,                 // 存在满足关节限位有效解
    NO_GEOM_SOLUTION = 1,   // 几何工作空间无解
    NO_VALID_SOLUTION = 2 // 几何有解，但全部候选解触碰关节限位
};

struct IK3Result
{
    std::vector<Eigen::Vector3d> candidates;
    IK3Status status;
};


Scorpion3AllJointsPos scorpion3_fk_all_joints(const Eigen::Vector3d& q, const Scorpion3Param& param);

Scorpion3FKRes scorpion3_fk(const Eigen::Vector3d& q, const Scorpion3Param& param);

inline double wrap_to_pi(double angle);

Eigen::Vector3d compute_error(const Scorpion3FKRes& y_act, double xd, double zd, double psid);

IK3Result scorpion3_ik(double xd, double zd, double psid, const Scorpion3Param& param);

Eigen::Vector3d select_nearest_solution(const std::vector<Eigen::Vector3d>& candidates,
                                         const Eigen::Vector3d& q_curr);


#endif //UMI_ON_AIR_ARM_ARM_3DOF_H
