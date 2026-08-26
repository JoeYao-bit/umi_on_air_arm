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

Eigen::Vector3d joint_angles{M_PI_2, 0., 0.}; // init angle

Canvas canvas("Coverage visualize", 20, 20, 10, 40);


int main()
{

//    std::cout << sqrt(pow(0.375, 2) + pow(0.06, 2)) << std::endl;

//    Eigen::Vector3d p_b = scorpion3_fk(init_angles);
//    std::cout<<"FK arm‑base pos: "<<p_b.transpose()<<"\n";

//    auto joints = scorpion3_fk_all_joints(q_test);
//
//    Eigen::Vector3d target{0.45, 0.0, 0.22};
//    Eigen::Vector3d q_guess{0,0,0};
//    Eigen::Vector3d q_sol = scorpion3_ik_nullspace(target, q_guess);
//    Eigen::Vector3d p_check = scorpion3_fk(q_sol);
//
//    std::cout<<"IK q [th1,th2,th3]: "<<q_sol.transpose()<<"\n";
//    std::cout<<"FK check pos: "<<p_check.transpose()<<"\n";
//    std::cout<<"position error: "<<(target-p_check).norm()<<" m\n";

    auto callback = [](int event, float x, float y, int flags, void *) {
        if(event == cv::EVENT_LBUTTONDOWN) {
            goal_pt[0] = x;
            goal_pt[1] = y;

            std::cout << "get goal point " << goal_pt << std::endl;

            goal_world = canvas.transformToWorld(goal_pt);

            std::cout << "get goal world point " << goal_world << std::endl;

            Eigen::Vector3d target(goal_world[0], 0, goal_world[1]);

            if(!scorpion3_check_reachable(goal_world[0], goal_world[1])) {
                std::cout << "goal " << goal_world << " out of reach !!!" << std::endl;
                return;
            }
            auto start_t = clock();
            joint_angles = scorpion3_ik_nullspace(target, joint_angles);
            std::cout << "ik cost " << (float)(clock() - start_t)/CLOCKS_PER_SEC << "s" << std::endl;
            Eigen::Vector3d p_check = scorpion3_fk(joint_angles);

            std::cout<<"position error: "<<(target-p_check).norm()<<" m\n";

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


//        auto origin = canvas.transformToPixel(0.f, 0.f);
//        canvas.drawCircleFloat(origin[0], origin[1], 0.05, false);
//
//        auto origin_x = canvas.transformToPixel(1.f, 0.f);
//        canvas.drawCircleFloat(origin_x[0], origin_x[1], 0.05, false, -1, COLOR_TABLE[0]);
//
//        auto origin_y = canvas.transformToPixel(0.f, 1.f);
//        canvas.drawCircleFloat(origin_y[0], origin_y[1], 0.05, false, -1, COLOR_TABLE[0]);

        auto joints = scorpion3_fk_all_joints(joint_angles);

        if(draw_bar) {
            for (int i = 0; i < joints.size() - 1; i++) {
                auto joint_1 = canvas.transformToPixel(joints[i].x(), joints[i].z());
                auto joint_2 = canvas.transformToPixel(joints[i + 1].x(), joints[i + 1].z());
                canvas.drawLineFloat(joint_1[0], joint_1[1], joint_2[0], joint_2[1], false, 1);
            }
        }
        if(draw_joint) {
            for (int i = 0; i < joints.size(); i++) {
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

    return 0;
}
