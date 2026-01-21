#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <my_robot_interfaces/msg/pose_target.hpp>
#include <my_robot_interfaces/msg/joint_pose.hpp>
#include <my_robot_interfaces/msg/open_close.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = my_robot_interfaces::msg::OpenClose;
using PoseValues = my_robot_interfaces::msg::PoseTarget;
using Joints = my_robot_interfaces::msg::JointPose;
using namespace std::placeholders;

class Commander{
public:
    Commander(std::shared_ptr<rclcpp::Node> node){
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
        gripper_->setMaxVelocityScalingFactor(1.0);
        gripper_->setMaxAccelerationScalingFactor(1.0);

        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper", 10, std::bind(&Commander::OpenGripperCallback, this, _1));
        
        pose_target_sub_ = node_->create_subscription<PoseValues>(
            "pose_target", 10, std::bind(&Commander::GoToPoseTargetCallback, this, _1));

        joint_target_sub_ = node_->create_subscription<Joints>(
            "joint_target", 10, std::bind(&Commander::GoToJointTargetCallback, this, _1));
    }

    struct PoseTargetArguments
        {
            double x, y, z;
            double roll, pitch, yaw;
        };

    void goToNamedTarget(const std::string &name){
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints){
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(const PoseTargetArguments &value, bool cartesian_path = false)
        {
        tf2::Quaternion q;
        q.setRPY(value.roll, value.pitch, value.yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = value.x;
        target_pose.pose.position.y = value.y;
        target_pose.pose.position.z = value.z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if(!cartesian_path){
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

            if (fraction == 1.0)
            {
                RCLCPP_INFO(node_->get_logger(), "Cartesian path computed successfully. Moving the arm.");
                arm_->execute(trajectory);
            }
        }    
    }

    void openGripper(){
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open_pose");
        planAndExecute(gripper_);
    }

    void closeGripper(){
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed_pose");
        planAndExecute(gripper_);
    }


private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface){
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(node_->get_logger(), "Planning successful, executing the plan.");
            interface->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed.");
        }
    }

    void OpenGripperCallback(const Bool &msg){
        if (msg.data) {
            openGripper();
        }
        else{
            closeGripper();
        }
    }

    void GoToPoseTargetCallback(const PoseValues &msg){
        PoseTargetArguments pose;
        pose.x = msg.x;
        pose.y = msg.y;
        pose.z = msg.z;
        pose.roll = msg.roll;
        pose.pitch = msg.pitch;
        pose.yaw = msg.yaw;
        
        goToPoseTarget(pose, msg.cartesian_path);
    }

    void GoToJointTargetCallback(const Joints &msg){
        goToJointTarget(msg.joints);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<PoseValues>::SharedPtr pose_target_sub_;
    rclcpp::Subscription<Joints>::SharedPtr joint_target_sub_;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}