#include "rclcpp/rclcpp.hpp"
#include "umi_arm_msg/msg/ik_input.hpp"
#include "umi_arm_msg/msg/ik_output.hpp"

#include "umi_arm_msg/msg/fk_input.hpp"
#include "umi_arm_msg/msg/fk_output.hpp"
#include "umi_arm_msg/msg/joint_angles.hpp"
#include "umi_arm_msg/msg/angle_control_feed_back.hpp"
#include "umi_arm_msg/msg/end_track.hpp"
#include "arm_3dof.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>   

using IKInput = umi_arm_msg::msg::IKInput; // 臂末端位姿逆运动学输入
using IKOutput = umi_arm_msg::msg::IKOutput; // 臂末端位姿逆运动学

using FKInput = umi_arm_msg::msg::FKInput; // 臂末端位姿正运动学输入
using FKOutput = umi_arm_msg::msg::FKOutput; // 臂末端位姿正运动学反馈

using JointAngles = umi_arm_msg::msg::JointAngles; // 当前关节角

using AngleControlFeedBack = umi_arm_msg::msg::AngleControlFeedBack; // 臂控制发布的执行关节角反馈

using EndTrack = umi_arm_msg::msg::EndTrack; // 目标臂末端轨迹，追踪目标

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
        RCLCPP_INFO(this->get_logger(), "Arm IK topic node ready. sub: %s , pub: %s", 
                    sub_ik_->get_topic_name(),pub_ik_->get_topic_name());

        sub_fk_ = this->create_subscription<FKInput>(
            "/arm_fk_in",
            rclcpp::QoS(10),
            std::bind(&ArmIkTopicNode::callbackFK, this, std::placeholders::_1)
        );
        pub_fk_ = this->create_publisher<FKOutput>("/arm_fk_out", rclcpp::QoS(10));
        RCLCPP_INFO(this->get_logger(), "Arm FK topic node ready. sub: %s , pub: %s", 
                    sub_fk_->get_topic_name(),pub_fk_->get_topic_name());

        sub_ja_ = this->create_subscription<JointAngles>(
            "/joint_angles",
            rclcpp::QoS(10),
            std::bind(&ArmIkTopicNode::callbackJA, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Arm Joint angle sub: %s", 
                    sub_ja_->get_topic_name());

        sub_acfb_ = this->create_subscription<AngleControlFeedBack>(
            "/angle_control_feedback",
            rclcpp::QoS(10),
            std::bind(&ArmIkTopicNode::callbackACFB, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Arm Angle control feedback sub: %s", 
                    sub_acfb_->get_topic_name());

        sub_et_ = this->create_subscription<EndTrack>(
            "/end_track",
            rclcpp::QoS(10),
            std::bind(&ArmIkTopicNode::callbackET, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Arm End track sub: %s", 
                    sub_et_->get_topic_name());


        // 10Hz 定时器：周期 100ms = 0.1s
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),   // 周期，100毫秒 =10Hz
            std::bind(&ArmIkTopicNode::timer_callback, this)
        );
    }

private:


    void timer_callback()
    {
        // ========== 10Hz 循环体在这里 ==========
        //RCLCPP_INFO(this->get_logger(), "10Hz tick"); // ok
    }

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

    void callbackJA(const JointAngles::SharedPtr msg) {
        q0_ = msg->q0;
        q1_ = msg->q1;
        q2_ = msg->q2;
        q3_ = msg->q3;

        RCLCPP_INFO(this->get_logger(),
                "Joint angles received: q0=%.3f, q1=%.3f, q2=%.3f, q3=%.3f", q0_, q1_, q2_, q3_);
    }

    void callbackACFB(const AngleControlFeedBack::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),
                "Angle control feedback received: error_code=%d at %d", msg->error_code, msg->header.stamp.sec);
    }

    void callbackET(const EndTrack::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(),
                "End track received: poses size=%zu at %d", msg->poses.size(), msg->header.stamp.sec);
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const auto& pose = msg->poses[i];
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)).getRPY(roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(),
                "Pose %zu: position=(%.3f, %.3f, %.3f), RPY=(%.3f, %.3f, %.3f)",
                i, pose.position.x, pose.position.y, pose.position.z,
                roll, pitch, yaw);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<IKInput>::SharedPtr sub_ik_;
    rclcpp::Publisher<IKOutput>::SharedPtr pub_ik_;

    rclcpp::Subscription<FKInput>::SharedPtr sub_fk_;
    rclcpp::Publisher<FKOutput>::SharedPtr pub_fk_;    

    rclcpp::Subscription<JointAngles>::SharedPtr sub_ja_;
    float q0_, q1_, q2_, q3_;

    rclcpp::Subscription<AngleControlFeedBack>::SharedPtr sub_acfb_;

    rclcpp::Subscription<EndTrack>::SharedPtr sub_et_;

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmIkTopicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
