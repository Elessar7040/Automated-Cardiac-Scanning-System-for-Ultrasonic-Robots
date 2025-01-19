/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 17:16:42
 * @LastEditors: “feiyang_hong” “feiyang.hong@infinityrobot.cn”
 * @LastEditTime: 2025-01-08 17:28:08
 * @FilePath: /planning_control_node/src/planning_node/test/cartesian_abs_action_client_test.cpp
 * @Description: 笛卡尔绝对规划测试
 */
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>

class CartesianMotionTest : public rclcpp::Node
{
public:
    CartesianMotionTest() : Node("moveit_cartesian_test")
    {
        // // 等待参数可用
        // while (!this->has_parameter("robot_description_semantic")) {
        //     RCLCPP_WARN(this->get_logger(), "等待 robot_description_semantic 参数...");
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        // }

        // // 获取参数
        // std::string srdf_string;
        // this->get_parameter("robot_description_semantic", srdf_string);
        
        // if (srdf_string.empty()) {
        //     RCLCPP_ERROR(this->get_logger(), "SRDF 内容为空!");
        //     return;
        // }

        // RCLCPP_INFO(this->get_logger(), "成功加载 SRDF");
        // 创建定时器，延迟2秒后开始运动，确保MoveIt完全启动
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&CartesianMotionTest::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "节点已启动，2秒后开始运动...");
    }

private:
    void timerCallback()
    {
        // 只执行一次，立即取消定时器
        timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "开始规划运动...");

        // // 创建move_group
        // moveit::planning_interface::MoveGroupInterface::Options opt("fairino5_group", "robot_description");  // arm_group是规划组名称
        // auto move_group = moveit::planning_interface::MoveGroupInterface(node, opt);
        // 使用Options方式创建move_group
        // moveit::planning_interface::MoveGroupInterface::Options options(
        //     shared_from_this(),   // 节点指针
        //     "fairino5_group",    // 规划组名称
        //     "robot_description", // 机器人描述参数名
            
        // );
        // auto move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(options);

        auto move_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_group");  // 替换为您的规划组名称

        RCLCPP_INFO(this->get_logger(), "move_group已创建...");

        // 设置规划参数
        move_group_ptr->setMaxVelocityScalingFactor(0.3);  // 设置最大速度为30%
        move_group_ptr->setMaxAccelerationScalingFactor(0.3);  // 设置最大加速度为30%
        move_group_ptr->setNumPlanningAttempts(10);  // 设置规划尝试次数
        move_group_ptr->setGoalPositionTolerance(0.01);  // 设置位置误差容许范围（米）
        // move_group_ptr->setGoalOrientationTolerance(0.01);  // 设置姿态误差容许范围（弧度）

        // 获取当前位姿
        geometry_msgs::msg::PoseStamped current_pose = move_group_ptr->getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "当前位置: x=%.3f, y=%.3f, z=%.3f",
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z);

        // 创建目标位姿
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.5;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.6;
        target_pose.orientation.w = 1.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;

        // 设置目标位姿
        move_group_ptr->setPoseTarget(target_pose);

        // 进行运动规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_ptr->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "规划成功，开始执行运动...");
            move_group_ptr->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "运动执行完成！");

            // 获取最终位姿
            geometry_msgs::msg::PoseStamped final_pose = move_group_ptr->getCurrentPose();
            RCLCPP_INFO(this->get_logger(), "最终位置: x=%.3f, y=%.3f, z=%.3f",
                final_pose.pose.position.x,
                final_pose.pose.position.y,
                final_pose.pose.position.z);
        } else {
            RCLCPP_ERROR(this->get_logger(), "运动规划失败！");
        }

        RCLCPP_INFO(this->get_logger(), "测试完成，按Ctrl+C退出...");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianMotionTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}