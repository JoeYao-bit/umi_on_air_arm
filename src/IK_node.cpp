#include "rclcpp/rclcpp.hpp"
#include "umi_arm_msg/msg/ik_input.hpp"
#include "umi_arm_msg/msg/ik_output.hpp"

#include "arm_3dof.h"

using IKInput = umi_arm_msg::msg::IKInput;
using IKOutput = umi_arm_msg::msg::IKOutput;

class ArmIkTopicNode : public rclcpp::Node
{
public:
    ArmIkTopicNode()
    : Node("arm_ik_topic_node")
    {
        sub_ = this->create_subscription<IKInput>(
            "/arm_ik_in",
            rclcpp::QoS(10),
            std::bind(&ArmIkTopicNode::callback, this, std::placeholders::_1)
        );

        pub_ = this->create_publisher<IKOutput>("/arm_ik_out", rclcpp::QoS(10));

        RCLCPP_INFO(this->get_logger(), "Arm IK topic node ready. sub: /arm_ik_in , pub: /arm_ik_out");
    }

private:

    void callback(const IKInput::SharedPtr msg)
    {
        Eigen::Vector3d q_init(msg->q0_init, msg->q1_init, msg->q2_init);
        Eigen::Vector3d p_des(msg->x_des, msg->y_des, msg->z_des);


        RCLCPP_INFO(this->get_logger(),
            "ik receve init joint angle[%.3f, %.3f, %.3f], targe position[%.3f, %.3f, %.3f]", q_init(0), q_init(1), q_init(2), p_des(0), p_des(1), p_des(2));
 

        Eigen::Vector3d q_sol = scorpion3_ik_nullspace(p_des, q_init);
        
        Eigen::Vector3d p_check = scorpion3_fk(q_sol);
        
        double residual = (p_des-p_check).norm();

        int ret_code;

        if(residual < 0.01) {
            ret_code = 0;
        } else {
            ret_code = 1;
        }

        IKOutput out_msg;
        out_msg.q0_target = q_sol(0);
        out_msg.q1_target = q_sol(1);
        out_msg.q2_target = q_sol(2);

        out_msg.error_code = ret_code;
        out_msg.residual = residual;

        pub_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(),
            "ik ret_code=%d residual=%.5f | joint_angle:[%.3f, %.3f, %.3f]",
            ret_code, residual, q_sol(0), q_sol(1), q_sol(2));
    }

    rclcpp::Subscription<IKInput>::SharedPtr sub_;
    rclcpp::Publisher<IKOutput>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmIkTopicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
