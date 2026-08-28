#include "rclcpp/rclcpp.hpp"
#include "umi_arm_msg/msg/ik_input.hpp"
#include "umi_arm_msg/msg/ik_output.hpp"

#include "umi_arm_msg/msg/fk_input.hpp"
#include "umi_arm_msg/msg/fk_output.hpp"

#include "arm_3dof.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using IKInput = umi_arm_msg::msg::IKInput;
using IKOutput = umi_arm_msg::msg::IKOutput;

using FKInput = umi_arm_msg::msg::FKInput;
using FKOutput = umi_arm_msg::msg::FKOutput;

Scorpion3Param param;

class ArmIkTopicNode : public rclcpp::Node
{
public:
    ArmIkTopicNode()
    : Node("arm_ik_topic_node")
    {
        sub_ik_ = this->create_subscription<IKInput>(
            "/arm_ik_in",
            rclcpp::QoS(10),
            std::bind(&ArmIkTopicNode::callbackIK, this, std::placeholders::_1)
        );
        pub_ik_ = this->create_publisher<IKOutput>("/arm_ik_out", rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(), "Arm IK topic node ready. sub: /arm_ik_in , pub: /arm_ik_out");
        sub_fk_ = this->create_subscription<FKInput>(
            "/arm_fk_in",
            rclcpp::QoS(10),
            std::bind(&ArmIkTopicNode::callbackFK, this, std::placeholders::_1)
        );
        pub_fk_ = this->create_publisher<FKOutput>("/arm_fk_out", rclcpp::QoS(10));
        RCLCPP_INFO(this->get_logger(), "Arm FK topic node ready. sub: /arm_fk_in , pub: /arm_fk_out");

    }

private:

    void callbackIK(const IKInput::SharedPtr msg)
    {
        Eigen::Vector3d q_init(msg->q0_init, msg->q1_init, msg->q2_init);
        Eigen::Vector3d p_des(msg->x_des, msg->y_des, msg->z_des);

        tf2::Quaternion tf_q;
        tf2::fromMsg(msg->quaternion, tf_q);
        double goal_roll, goal_pitch, goal_yaw;
        tf2::Matrix3x3(tf_q).getRPY(goal_roll, goal_pitch, goal_yaw);


        RCLCPP_INFO(this->get_logger(),
            "ik receve init joint angle[%.3f, %.3f, %.3f], targe position[%.3f, %.3f, %.3f], target pitch[%.3f], target RPY[%.3f, %.3f, %.3f]", q_init(0), q_init(1), q_init(2), p_des(0), p_des(1), p_des(2), goal_pitch, goal_roll, goal_yaw);


        auto ik_res = scorpion3_ik(p_des(0), p_des(2), goal_pitch, param);
        if(ik_res.status == IK3Status::OK)
        {

            Eigen::Vector3d q_sol = select_nearest_solution(ik_res.candidates, q_init);
            auto fk_res = scorpion3_fk(q_sol, param);
            Eigen::Vector3d err = compute_error(fk_res, p_des(0), p_des(2), goal_pitch);
            double error = err.norm();
            std::cout << "\nIK‑FK error:" << err.transpose() << "\n";
            IKOutput out_msg;
            out_msg.q0_target = q_sol(0);
            out_msg.q1_target = q_sol(1);
            out_msg.q2_target = q_sol(2);

            out_msg.q3_target = goal_roll; // for end of arm's wrist roll joint

            out_msg.error_code = static_cast<int>(ik_res.status);
            out_msg.residual = error;

            pub_ik_->publish(out_msg);

            RCLCPP_INFO(this->get_logger(),
                "ik ret_code=%d error=%.5f | joint_angle:[%.3f, %.3f, %.3f, %.3f]",
                ik_res.status, error, q_sol(0), q_sol(1), q_sol(2), goal_roll);
        } else if(ik_res.status == IK3Status::NO_GEOM_SOLUTION) {
            // 目标超出臂工作空间，几何上就不可能到达
            std::cout << "目标超出臂工作空间，几何上就不可能到达" << std::endl;
            IKOutput out_msg;
            out_msg.error_code = static_cast<int>(ik_res.status);
            pub_ik_->publish(out_msg);

        } else if(ik_res.status == IK3Status::NO_VALID_SOLUTION) {
            // 几何上能到，但关节限位卡死，物理硬件无法实现
            std::cout << "几何上能到，但关节限位卡死，物理硬件无法实现" << std::endl;
            IKOutput out_msg;
            out_msg.error_code = static_cast<int>(ik_res.status);
            pub_ik_->publish(out_msg);
        }


    }

    void callbackFK(const FKInput::SharedPtr msg) {
        Eigen::Vector3d q_input(msg->q0, msg->q1, msg->q2);
        auto fk_res = scorpion3_fk(q_input, param);

        RCLCPP_INFO(this->get_logger(),
                "fk receive q0,q1,q2,q3=[%.3f, %.3f, %.3f, %.3f]", msg->q0, msg->q1, msg->q2, msg->q3);

        FKOutput out_msg;
        out_msg.x = fk_res.x;
        out_msg.y = 0.0; // Assuming y is always 0 in this 2D arm model
        out_msg.z = fk_res.z;
        
        tf2::Quaternion tf_q;
        tf_q.setRPY(msg->q3, fk_res.psi, 0);   // RPY转四元数, yaw = 0
        out_msg.quaternion = tf2::toMsg(tf_q);

        pub_fk_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(),
                "fk output x,y,z=[%.3f, %.3f, %.3f], RPY=[%.3f, %.3f, %.3f]",
                out_msg.x, out_msg.y, out_msg.z, fk_res.psi, msg->q3, 0);

    }


    rclcpp::Subscription<IKInput>::SharedPtr sub_ik_;
    rclcpp::Publisher<IKOutput>::SharedPtr pub_ik_;

    rclcpp::Subscription<FKInput>::SharedPtr sub_fk_;
    rclcpp::Publisher<FKOutput>::SharedPtr pub_fk_;    
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmIkTopicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
