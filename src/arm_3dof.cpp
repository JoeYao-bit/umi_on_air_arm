//
// Created by yaozhuo on 8/26/26.
//

#include "arm_3dof.h"
#include <cmath>
#include <iostream>

Eigen::Vector3d scorpion3_fk(const Eigen::Vector3d& q, const Scorpion3Param& param)
{
    double th1 = q(0);
    double th2 = q(1);
    double th3 = q(2);

    double phi1 = th1;
    double phi2 = th1 + th2;
    double phi3 = th1 + th2 + th3;

    double x = param.l1 * std::cos(phi1)
               + param.l2 * std::cos(phi2)
               + param.l3 * std::cos(phi3);

    double z = param.l1 * std::sin(phi1)
               + param.l2 * std::sin(phi2)
               + param.l3 * std::sin(phi3);

    return Eigen::Vector3d{x, 0.0, z};
}


std::vector<Eigen::Vector3d> scorpion3_fk_all_joints(const Eigen::Vector3d& q, const Scorpion3Param& param)
{
    double th1 = q(0);
    double th2 = q(1);
    double th3 = q(2);

    double phi1 = th1;
    double phi2 = th1 + th2;
    double phi3 = th1 + th2 + th3;

    std::vector<Eigen::Vector3d> pts;

    // base：臂安装基座原点
    Eigen::Vector3d base{0.0, 0.0, 0.0};
    pts.push_back(base);

    // J2关节位置：J1 + l1 沿 phi1 方向
    Eigen::Vector3d j2;
    j2.x() = param.l1 * std::cos(phi1);
    j2.y() = 0.0;
    j2.z() = param.l1 * std::sin(phi1);
    pts.push_back(j2);

    // J3关节位置：J2 + l2 沿 phi2 方向
    Eigen::Vector3d j3;
    j3.x() = param.l1 * std::cos(phi1) + param.l2 * std::cos(phi2);
    j3.y() = 0.0;
    j3.z() = param.l1 * std::sin(phi1) + param.l2 * std::sin(phi2);
    pts.push_back(j3);

    // 末端执行器 ee：J3 + L3g() 沿 phi3 方向
    Eigen::Vector3d ee;
    ee.x() = param.l1 * std::cos(phi1) + param.l2 * std::cos(phi2) + param.l3 * std::cos(phi3);
    ee.y() = 0.0;
    ee.z() = param.l1 * std::sin(phi1) + param.l2 * std::sin(phi2) + param.l3 * std::sin(phi3);
    pts.push_back(ee);

    return pts;
}

Eigen::Matrix<double,3,3> scorpion3_jacobian(const Eigen::Vector3d& q, const Scorpion3Param& param)
{
    Eigen::Matrix<double,3,3> J;
    J.setZero();

    double th1 = q(0);
    double th2 = q(1);
    double th3 = q(2);

    double phi1 = th1;
    double phi2 = th1 + th2;
    double phi3 = th1 + th2 + th3;

    // ∂/∂ th1
    double dx1 = -param.l1*std::sin(phi1) - param.l2*std::sin(phi2) - param.l3*std::sin(phi3);
    double dz1 =  param.l1*std::cos(phi1) + param.l2*std::cos(phi2) + param.l3*std::cos(phi3);

    // ∂/∂ th2
    double dx2 = -param.l2*std::sin(phi2) - param.l3*std::sin(phi3);
    double dz2 =  param.l2*std::cos(phi2) + param.l3*std::cos(phi3);

    // ∂/∂ th3
    double dx3 = -param.l3*std::sin(phi3);
    double dz3 =  param.l3*std::cos(phi3);

    J.row(0) << dx1, dx2, dx3;
    J.row(1).setZero();
    J.row(2) << dz1, dz2, dz3;

    return J;
}


Eigen::Vector3d scorpion3_ik_nullspace(const Eigen::Vector3d& p_des,
                                       const Eigen::Vector3d& q_init,
                                       int max_iter,
                                       double tol,
                                       double task_gain,
                                       double null_gain,
                                       const Scorpion3Param& param)
{
    Eigen::Vector3d q = q_init;
    const double lambda = 1e-3; // damping
    int iter;
    for(iter = 0; iter < max_iter; ++iter)
    {
        Eigen::Vector3d p_act = scorpion3_fk(q, param);
        Eigen::Vector3d e = p_des - p_act;
        // y方向误差直接丢弃，臂无法产生y运动
        e(1) = 0.0;

        if(e.norm() < tol) {
            std::cout << "reach goal position" << std::endl;
            break;
        }

        Eigen::Matrix<double,3,3> J = scorpion3_jacobian(q, param);
        // Damped Moore‑Penrose伪逆
        Eigen::Matrix<double,3,3> Jt = J.transpose();
        Eigen::Matrix<double,3,3> JJt = J*Jt + lambda*Eigen::Matrix<double,3,3>::Identity();
        Eigen::Matrix<double,3,3> Jpinv = Jt * JJt.inverse();

        // 任务空间速度
        Eigen::Vector3d dq_task = task_gain * Jpinv * e;

        // 零空间投影算子 P = I - J⁺J
        Eigen::Matrix<double,3,3> P = Eigen::Matrix<double,3,3>::Identity() - Jpinv * J;
        // 零空间梯度：向q_init回归
        Eigen::Vector3d dq_null = null_gain * P * (q_init - q);

        Eigen::Vector3d dq = dq_task + dq_null;
        q += dq;

        // 关节限位
        q = q.array().max(param.q_min.array()).min(param.q_max.array());

    }
    if(iter == max_iter) {
        std::cout << "reach max iter " << max_iter << std::endl;
    }
    return q;
}

Eigen::MatrixXd compute_jacobian_3r(const Eigen::Vector3d& q, const Scorpion3Param& param)
{
    double th1 = q(0);
    double th2 = q(1);
    double th3 = q(2);

    double phi1 = th1;
    double phi2 = th1 + th2;
    double phi3 = th1 + th2 + th3;

    double l1 = param.l1;
    double l2 = param.l2;
    double l3 = param.l3;

    Eigen::MatrixXd J(2,3);

    J(0,0) = -l1*sin(phi1) - l2*sin(phi2) - l3*sin(phi3);
    J(0,1) = -l2*sin(phi2) - l3*sin(phi3);
    J(0,2) = -l3*sin(phi3);

    J(1,0) =  l1*cos(phi1) + l2*cos(phi2) + l3*cos(phi3);
    J(1,1) =  l2*cos(phi2) + l3*cos(phi3);
    J(1,2) =  l3*cos(phi3);

    return J;
}

/**
 * @brief 零空间逆解：满足末端位置前提下尽量不偏离q_init
 * @param p_des 目标末端 (x,0,z)
 * @param q_init 初始关节角；无解时直接返回该值
 * @param max_iter 最大迭代步数
 * @param tol 收敛阈值(m)
 * @param task_gain 任务增益
 * @param null_gain 零空间增益，建议0.1~0.3
 * @param param 机械臂参数（包含连杆+关节限位）
 * @return 收敛返回解；不可达/奇异/不收敛直接返回 q_init
 */
//Eigen::Vector3d scorpion3_ik_nullspace(const Eigen::Vector3d& p_des,
//                                       const Eigen::Vector3d& q_init,
//                                       int max_iter,
//                                       double tol,
//                                       double task_gain,
//                                       double null_gain,
//                                       const Scorpion3Param& param)
//{
//    Eigen::Vector2d xd(p_des(0), p_des(2));
//
//    if (!scorpion3_check_reachable(p_des(0), p_des(2), param))
//    {
//        std::cout << "not reach able (" << p_des(0) << ", " << p_des(2) << ")" << std::endl;
//        return q_init;
//    }
//
//    Eigen::Vector3d q = q_init;
//
//    for (int iter = 0; iter < max_iter; ++iter)
//    {
//        Eigen::Vector3d fk_p = scorpion3_fk(q, param);
//        Eigen::Vector2d fk_xy(fk_p(0), fk_p(2));
//        Eigen::Vector2d e = xd - fk_xy;
//
//        double res = e.norm();
//        if (res < tol)
//        {
//            // error is small, get success solution
//            return q;
//        }
//
//        Eigen::MatrixXd J = compute_jacobian_3r(q, param);
//
//        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
//        const auto& svals = svd.singularValues();
//        double sigma_min = svals(1);
//        if (sigma_min < 1e-6)
//        {
//            // > 奇异值物理含义：
//            //
//            //- **大奇异值**：该方向上，很小的关节转动，就能产生明显的末端移动。
//            //- **最小奇异值 \(\sigma_{min}\)**：代表**最弱的运动方向**。
//            //\(\sigma_{min}\) 越接近 0，代表这个方向几乎动不了
//            std::cout << "sigma_min < 1e-6" << std::endl;
//            return q_init;
//        }
//
//        // 手动组装伪逆 V * Σ⁻¹ * Uᵀ
//        Eigen::MatrixXd sigma_inv(svals.size(), svals.size());
//        sigma_inv.setZero();
//        for(int i=0; i<svals.size(); ++i)
//        {
//            sigma_inv(i,i) = 1.0 / svals(i);
//        }
//        Eigen::MatrixXd J_pinv = svd.matrixV() * sigma_inv * svd.matrixU().transpose();
//
//        Eigen::Matrix3d N = Eigen::Matrix3d::Identity() - J_pinv * J;
//
//        Eigen::Vector3d dq_task = J_pinv * e;
//        Eigen::Vector3d grad_H = q - q_init;
//        Eigen::Vector3d dq_null = -grad_H;
//
//        Eigen::Vector3d dq = task_gain * dq_task + null_gain * (N * dq_null);
//        q += dq;
//
//        q = q.array().max(param.q_min.array()).min(param.q_max.array());
//    }
//
//    return q_init;
//}

/**
 * @brief 快速预检查：末端目标点是否几何可达（不考虑关节角度限位）
 * @param xd 目标末端 [x, z]
 * @param param 臂连杆参数
 * @param eps 数值容差，处理浮点误差
 * @return true：点在工作空间圆盘/圆环内，可进入IK求解；false：完全不可达
 */
bool scorpion3_check_reachable(double x, double z,
                               const Scorpion3Param& param,
                               double eps)
{
    // J1 关节位置 (0, l0)
    double dx = x;
    double dz = z;
    double rho = std::hypot(dx, dz);

    const double l1 = param.l1;
    const double l2 = param.l2;
    const double l3 = param.l3;

    const double Rmax = l1 + l2 + l3;
    const double Rmin = std::max(0.0, l1 - (l2 + l3));

    bool in_range = (rho >= Rmin - eps) && (rho <= Rmax + eps);
    //std::cout << " rho >= Rmin - eps " << rho << " >= " << Rmin - eps << " / rho <= Rmax + eps = " << rho << " <= " << Rmax + eps << std::endl;
    return in_range;
}