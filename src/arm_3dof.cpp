//
// Created by yaozhuo on 8/26/26.
//

#include "arm_3dof.h"


/**
 * @brief 本构型专用解析逆解：目标 xd, zd, psid
 * @param xd 末端x
 * @param zd 末端z（含基座l0偏移）
 * @param psid 期望末端俯仰角 rad
 * @param param 臂参数
 * @return 候选解集合 0/1/2组；status标记求解状态
 */
IK3Result scorpion3_ik(double xd, double zd, double psid, const Scorpion3Param& param)
{
    IK3Result res{};
    const double L3g = param.L3g();
    const double l0  = param.l0;
    const double l1  = param.l1;
    const double l2  = param.l2;
    const double jmin = -M_PI / 2.0;
    const double jmax =  M_PI / 2.0;
    const double eps  = 1e-8;

    // 扣除末端连杆，退化为标准2‑R逆问题
    double x_p = xd - L3g * std::cos(psid);
    double z_p = zd - l0 - L3g * std::sin(psid);

    double r2 = x_p*x_p + z_p*z_p;
    double r  = std::sqrt(r2);

    // -------- 1.几何可达性判定 --------
    if (r > l1 + l2 + eps || r < std::fabs(l1 - l2) - eps)
    {
        res.status = IK3Status::NO_GEOM_SOLUTION;
        return res;
    }

    // -------- 2.计算两组几何解 elbow‑up / elbow‑down --------
    double c2 = (r2 - l1*l1 - l2*l2) / (2.0 * l1 * l2);
    c2 = std::clamp(c2, -1.0, 1.0);
    double theta2_pos = std::acos(c2);
    double theta2_list[2] = {theta2_pos, -theta2_pos};

    for (double theta2 : theta2_list)
    {
        double K1 = l1 + l2 * std::cos(theta2);
        double K2 = l2 * std::sin(theta2);

        double phi_r = std::atan2(z_p, x_p);
        double phi_k = std::atan2(K2, K1);
        double theta1 = phi_r - phi_k;
        double theta3 = psid - theta1 - theta2;

        Eigen::Vector3d q{theta1, theta2, theta3};

        // 校验关节限位
        bool valid = true;
        for(int i=0;i<3;i++)
        {
            if(q(i) < jmin-eps || q(i) > jmax+eps)
            {
                valid = false;
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
    }
    else
    {
        res.status = IK3Status::OK;
    }
    return res;
}

/**
 * @brief 从候选解中选取距离当前关节最近的解（用于运行时平滑）
 * @param candidates 候选解
 * @param q_curr 当前实际关节角
 * @return 最优解；候选为空时返回零向量
 */
Eigen::Vector3d select_nearest_solution(const std::vector<Eigen::Vector3d>& candidates,
                                         const Eigen::Vector3d& q_curr)
{
    if(candidates.empty()) return Eigen::Vector3d::Zero();
    size_t best_idx = 0;
    double min_sq = (candidates[0]-q_curr).squaredNorm();
    for(size_t i=1;i<candidates.size();i++)
    {
        double d2 = (candidates[i]-q_curr).squaredNorm();
        if(d2 < min_sq)
        {
            min_sq = d2;
            best_idx = i;
        }
    }
    return candidates[best_idx];
}


Scorpion3FKRes scorpion3_fk(const Eigen::Vector3d& q, const Scorpion3Param& param)
{
    double th1 = q(0);
    double th2 = q(1);
    double th3 = q(2);
    double phi1 = th1;
    double phi2 = th1 + th2;
    double phi3 = th1 + th2 + th3;

    const double L3g = param.L3g();
    double x = param.l1 * std::cos(phi1)
             + param.l2 * std::cos(phi2)
             + L3g * std::cos(phi3);

    double z = param.l0
             + param.l1 * std::sin(phi1)
             + param.l2 * std::sin(phi2)
             + L3g * std::sin(phi3);

    double psi = phi3;
    return {x,z,psi};
}
