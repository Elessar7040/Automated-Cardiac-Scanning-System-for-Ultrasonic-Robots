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
        // 延迟初始化 move_group_ptr_
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 500ms后初始化
            std::bind(&CartesianMotionTest::init_move_group, this));

        RCLCPP_INFO(this->get_logger(), "节点已启动，等待初始化...");
    }

private:
    void init_move_group()
    {
        // 只执行一次
        timer_->cancel();

        // 在这里初始化 move_group_ptr_
        move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_group");

        // 创建订阅者
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&CartesianMotionTest::pose_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "笛卡尔运动节点初始化完成，等待目标位姿...");
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        RCLCPP_INFO(this->get_logger(), "收到新的目标位姿，开始规划...");

        move_group_ptr_->setMaxVelocityScalingFactor(0.3);

        // 获取当前位姿
        geometry_msgs::msg::PoseStamped current_pose = move_group_ptr_->getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "当前位置: x=%.3f, y=%.3f, z=%.3f",
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z);

        // // 创建目标位姿
        // geometry_msgs::msg::Pose target_pose;
        // target_pose.position.x = 0.3;
        // target_pose.position.y = 0.2;
        // target_pose.position.z = 0.7;
        // target_pose.orientation.w = 1.0;
        // target_pose.orientation.x = 0.0;
        // target_pose.orientation.y = 0.0;
        // target_pose.orientation.z = 0.0;

        // 设置目标位姿
        move_group_ptr_->setPoseTarget(msg->pose);

        // 进行运动规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_ptr_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "规划成功，开始执行运动...");
            move_group_ptr_->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "运动执行完成！");

            // 获取最终位姿
            geometry_msgs::msg::PoseStamped final_pose = move_group_ptr_->getCurrentPose();
        } else {
            RCLCPP_ERROR(this->get_logger(), "运动规划失败！");
        }

    }

    // 添加 timer_ 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianMotionTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}