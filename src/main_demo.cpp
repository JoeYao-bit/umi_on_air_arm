//
// Created by yaozhuo on 8/26/26.
//

#include <iostream>
#include "arm_3dof.h"
#include "freeNav-base/visualization/canvas/canvas.h"

using namespace freeNav;

Pointf<2> goal_pt{0, 0};
Pointf<2> goal_world;
bool draw_joint = true;
bool draw_bar = true;
bool draw_goal = true;

Eigen::Vector3d joint_angles{0., 0., 0.}; // init angle

Canvas canvas("UMI-Arm visualize", 20, 20, 10, 40);

Scorpion3Param param;

double psid = 0.0; // target pitch

int main()
{
    // Eigen::Vector3d q_zero{0.0,0.0,0.0};
    // auto all_pos = scorpion3_fk_all_joints(q_zero, param);

    // std::cout << "==== q = [0,0,0] test (全部水平向右) ====\n";
    // std::cout << "J1: " << all_pos.J1.transpose() << "\n";
    // std::cout << "J2: " << all_pos.J2.transpose() << "\n";
    // std::cout << "J3: " << all_pos.J3.transpose() << "\n";
    // std::cout << "EE: " << all_pos.EE.transpose() << "\n";
    // std::cout << "psi:" << all_pos.psi << "\n";

    // // IK‑FK闭环验证
    // double xd = 0.45;
    // double zd = 0.10;
    // double psid = 0.15;
    // auto ik_res = scorpion3_ik(xd, zd, psid, param);
    // if(ik_res.status == IK3Status::OK)
    // {
    //     Eigen::Vector3d q_curr{0,0,0};
    //     Eigen::Vector3d q_sol = select_nearest_solution(ik_res.candidates, q_curr);
    //     joint_angles = q_sol;
    //     auto fk_res = scorpion3_fk(q_sol, param);
    //     Eigen::Vector3d err = compute_error(fk_res, xd, zd, psid);
    //     std::cout << "\nIK‑FK error:" << err.transpose() << "\n";
    // } else if(ik_res.status == IK3Status::NO_GEOM_SOLUTION) {
    //     // 目标超出臂工作空间，几何上就不可能到达
    //     std::cout << "目标超出臂工作空间，几何上就不可能到达" << std::endl;
    // } else if(ik_res.status == IK3Status::NO_VALID_SOLUTION) {
    //     // 几何上能到，但关节限位卡死，物理硬件无法实现
    //     std::cout << "几何上能到，但关节限位卡死，物理硬件无法实现" << std::endl;
    // }
    // return 0;

    auto callback = [](int event, float x, float y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            goal_pt[0] = x;
            goal_pt[1] = y;

            std::cout << "get goal point " << goal_pt << std::endl;

            goal_world = canvas.transformToWorld(goal_pt);

            std::cout << "get goal world point " << goal_world << std::endl;

            Eigen::Vector3d target(goal_world[0], 0, goal_world[1]);

            auto start_t = clock();

            auto ik_res = scorpion3_ik(target.x(), target.z(), psid, param);
            if(ik_res.status == IK3Status::OK)
            {

                Eigen::Vector3d q_sol = select_nearest_solution(ik_res.candidates, joint_angles);
                joint_angles = q_sol;
                auto fk_res = scorpion3_fk(q_sol, param);
                Eigen::Vector3d err = compute_error(fk_res, target.x(), target.z(), psid);
                std::cout << "\nIK‑FK error:" << err.transpose() << "\n";
            } else if(ik_res.status == IK3Status::NO_GEOM_SOLUTION) {
                // 目标超出臂工作空间，几何上就不可能到达
                std::cout << "目标超出臂工作空间，几何上就不可能到达" << std::endl;
            } else if(ik_res.status == IK3Status::NO_VALID_SOLUTION) {
                // 几何上能到，但关节限位卡死，物理硬件无法实现
                std::cout << "几何上能到，但关节限位卡死，物理硬件无法实现" << std::endl;
            }
            std::cout << "ik cost " << (float)(clock() - start_t)/CLOCKS_PER_SEC << "s" << std::endl;
        }
    };

    canvas.setMouseCallBack(callback);

    while(true) {
        canvas.resetCanvas();
        canvas.drawEmptyGrid();

        // draw axis
        Pointf<2> axis_x1 = canvas.transformToPixel(-1.0f, 0.f);
        Pointf<2> axis_x2 = canvas.transformToPixel(1.f, .0f);

        canvas.drawArrowFloat(axis_x1[0], axis_x1[1], axis_x2[0], axis_x2[1], 1, false);

        Pointf<2> axis_y1 = canvas.transformToPixel(0.f, -1.f);
        Pointf<2> axis_y2 = canvas.transformToPixel(0.f, 1.0f);

        canvas.drawArrowFloat(axis_y1[0], axis_y1[1], axis_y2[0], axis_y2[1], 1, false);

        auto joint_and_pitch = scorpion3_fk_all_joints(joint_angles, param);

        std::vector<Eigen::Vector3d> joints = {joint_and_pitch.J1, joint_and_pitch.J2, joint_and_pitch.J3, joint_and_pitch.EE};

        if(draw_bar) {
            for (int i = 0; i < joints.size() - 1; i++) {
                auto joint_1 = canvas.transformToPixel(joints[i].x(), joints[i].z());
                auto joint_2 = canvas.transformToPixel(joints[i + 1].x(), joints[i + 1].z());
                canvas.drawLineFloat(joint_1[0], joint_1[1], joint_2[0], joint_2[1], false, 1);
            }
        }
        if(draw_joint) {
            for (int i = 0; i < joints.size()-1; i++) {
                auto joint = canvas.transformToPixel(joints[i].x(), joints[i].z());
                canvas.drawCircleFloat(joint[0], joint[1], 0.2, false, -1, COLOR_TABLE[0]);
            }
        }
        if(draw_goal) {
            auto goal = canvas.transformToPixel(goal_world[0], goal_world[1]);
            canvas.drawCircleFloat(goal[0], goal[1], 0.3, false, 1, COLOR_TABLE[1]);
            // draw available goal range
            auto origin = canvas.transformToPixel(0, 0);
            Scorpion3Param param;
            float r = param.l1 + param.l2 + param.l3;
            canvas.drawCircleFloat(origin[0], origin[1], r*canvas.resolution_, false, 1, COLOR_TABLE[1]);
        }


        char key = canvas.show(500);
        switch (key) {
            case 'j':
                draw_joint = !draw_joint;
                break;
            case 'b':
                draw_bar = !draw_bar;
                break;
            case 'g':
                draw_goal = !draw_goal;
            default:
                break;
        }
    }

//     return 0;
}
