//
// Created by yaozhuo on 8/26/26.
//

#include "arm_3dof.h"

Scorpion3AllJointsPos scorpion3_fk_all_joints(const Eigen::Vector3d& q, const Scorpion3Param& param)
{
    Scorpion3AllJointsPos res;
    double th1 = q(0);
    double th2 = q(1);
    double th3 = q(2);

    double phi1 = th1;
    double phi2 = th1 + th2;
    double phi3 = th1 + th2 + th3;
    res.psi = phi3;

    res.J1 = Eigen::Vector3d(0.0, 0.0, 0.0);

    // 杆1：零位竖直向上，保持不变
    res.J2 = res.J1 + Eigen::Vector3d(
        param.l1 * std::sin(phi1),
        0.0,
        param.l1 * std::cos(phi1)
    );

    // =========杆2：零位沿+X，z增加负号 -sin(phi2) =========
    res.J3 = res.J2 + Eigen::Vector3d(
        param.l2 * std::cos(phi2),
        0.0,
        -param.l2 * std::sin(phi2)
    );

    // =========杆3：零位沿+X，z增加负号 -sin(phi3) =========
    res.EE = res.J3 + Eigen::Vector3d(
        param.l3 * std::cos(phi3),
        0.0,
        -param.l3 * std::sin(phi3)
    );
    return res;
}

Scorpion3FKRes scorpion3_fk(const Eigen::Vector3d& q, const Scorpion3Param& param)
{
    auto all = scorpion3_fk_all_joints(q, param);
    return {all.EE.x(), all.EE.z(), all.psi};
}

inline double wrap_to_pi(double angle)
{
    return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

Eigen::Vector3d compute_error(const Scorpion3FKRes& y_act, double xd, double zd, double psid)
{
    Eigen::Vector3d e;
    e(0) = xd - y_act.x;
    e(1) = zd - y_act.z;
    e(2) = wrap_to_pi(psid - y_act.psi);
    return e;
}

IK3Result scorpion3_ik(double xd, double zd, double psid, const Scorpion3Param& param)
{
    IK3Result res{};
    const double l1 = param.l1;
    const double l2 = param.l2;
    const double l3 = param.l3;
    const double eps = 1e-8;

    // 杆3：dx=l3 cos(psid), dz= -l3 sin(psid)
    double x_p = xd - l3 * std::cos(psid);
    double z_p = zd - (-l3 * std::sin(psid));

    double d_sq = x_p*x_p + z_p*z_p;
    double d = std::sqrt(d_sq);

    if(d > l1 + l2 + eps || d < std::fabs(l1 - l2) - eps)
    {
        res.status = IK3Status::NO_GEOM_SOLUTION;
        return res;
    }

    double a = (l1*l1 - l2*l2 + d_sq) / (2.0*d);
    double h_sq = l1*l1 - a*a;
    if(h_sq < -eps)
    {
        res.status = IK3Status::NO_GEOM_SOLUTION;
        return res;
    }
    if(h_sq < 0.0) h_sq = 0.0;
    double h = std::sqrt(h_sq);

    double xm = a * x_p / d;
    double zm = a * z_p / d;
    double rx = -z_p * (h/d);
    double rz = x_p * (h/d);

    std::vector<Eigen::Vector2d> j2_candidates;
    j2_candidates.emplace_back(xm + rx, zm + rz);
    j2_candidates.emplace_back(xm - rx, zm - rz);

    for(auto const& j2 : j2_candidates)
    {
        double x_j2 = j2(0);
        double z_j2 = j2(1);
        double th1 = std::atan2(x_j2, z_j2);

        double vx = x_p - x_j2;
        double vz = z_p - z_j2;
        // 杆2世界向量：vx = l2 cosφ2，vz = -l2 sinφ2
        // tanφ2 = -vz / vx
        double phi2 = std::atan2(-vz, vx);

        double th2 = phi2 - th1;
        double th3 = psid - th1 - th2;

        Eigen::Vector3d q{th1, th2, th3};

        bool valid = true;
        for(int i=0; i<3; i++)
        {
            if(q(i) < param.q_min[i] - eps || q(i) > param.q_max[i] + eps)
            {
                valid = false;
                // std::cout<<"越限 q["<<i<<"]="<<q(i)<<" min="<<param.q_min[i]<<" max="<<param.q_max[i]<<"\n";
                break;
            }
        }
        if(valid)
        {
            res.candidates.push_back(q);
        }
    }

    if(res.candidates.empty())
    {
        res.status = IK3Status::NO_VALID_SOLUTION;
    }else{
        res.status = IK3Status::OK;
    }
    return res;
}


Eigen::Vector3d select_nearest_solution(const std::vector<Eigen::Vector3d>& candidates,
                                        const Eigen::Vector3d& q_curr)
{
    if (candidates.empty())
        return Eigen::Vector3d::Zero();

    size_t best_idx = 0;
    double min_sq = (candidates[0] - q_curr).squaredNorm();
    for (size_t i = 1; i < candidates.size(); i++)
    {
        double d2 = (candidates[i] - q_curr).squaredNorm();
        if (d2 < min_sq)
        {
            min_sq = d2;
            best_idx = i;
        }
    }
    return candidates[best_idx];
}


// ---------------- 测试示例 main ----------------
/*
#include <iostream>
int main()
{
    Scorpion3Param param;
    Eigen::Vector3d q_zero{0.0,0.0,0.0};
    auto all_pos = scorpion3_fk_all_joints(q_zero, param);

    std::cout << "==== q = [0,0,0] test (全部水平向右) ====\n";
    std::cout << "J1: " << all_pos.J1.transpose() << "\n";
    std::cout << "J2: " << all_pos.J2.transpose() << "\n";
    std::cout << "J3: " << all_pos.J3.transpose() << "\n";
    std::cout << "EE: " << all_pos.EE.transpose() << "\n";
    std::cout << "psi:" << all_pos.psi << "\n";

    // IK‑FK闭环验证
    double xd = 0.45;
    double zd = 0.10;
    double psid = 0.15;
    auto ik_res = scorpion3_ik(xd, zd, psid, param);
    if(ik_res.status == IK3Status::OK)
    {
        Eigen::Vector3d q_curr{0,0,0};
        Eigen::Vector3d q_sol = select_nearest_solution(ik_res.candidates, q_curr);
        auto fk_res = scorpion3_fk(q_sol, param);
        Eigen::Vector3d err = compute_error(fk_res, xd, zd, psid);
        std::cout << "\nIK‑FK error:" << err.transpose() << "\n";
    }
    return 0;
}
*/














