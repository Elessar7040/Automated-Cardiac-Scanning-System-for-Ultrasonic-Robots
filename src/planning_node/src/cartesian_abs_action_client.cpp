/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-02 16:25:10
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-07 10:50:54
 * @FilePath: /planning_control_node/src/planning_node/src/cartesian_abs_action_client_2.cpp
 * @Description: 笛卡尔空间绝对位置运动的Action客户端。
 *               订阅话题 "target_pose"，接收目标位姿信息；
 *               通过Action接口 "moveEndToAbsPos" 发送运动请求；
 *               实现机械臂末端执行器移动到笛卡尔空间绝对位置。
 * 
 * 订阅话题:
 *   - target_pose (geometry_msgs/msg/PoseStamped): 目标位姿
 * 
 * Action 客户端:
 *   - moveEndToAbsPos: 用于发送绝对位置运动请求
 *     - 输入: arm_id (机械臂ID), pos (目标位姿)
 *     - 输出: success (执行结果), message (结果信息)
 */

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "planning_node/action/move_end_to_abs_pos.hpp"

class CartesianActionClient : public rclcpp::Node
{
public:
    using MoveEndToAbsPos = planning_node::action::MoveEndToAbsPos;
    using GoalHandleMoveEndToAbsPos = rclcpp_action::ClientGoalHandle<MoveEndToAbsPos>;

    CartesianActionClient() : Node("cartesian_abs_action_client")
    {
        // 创建 Action 客户端
        client_ptr_ = rclcpp_action::create_client<MoveEndToAbsPos>(
            this, "moveEndToAbsPos");

        // 创建目标位姿订阅者
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&CartesianActionClient::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "笛卡尔绝对运动客户端已启动，等待目标位姿...");
    }

    ~CartesianActionClient() = default;

private:
    rclcpp_action::Client<MoveEndToAbsPos>::SharedPtr client_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到新的目标位姿，发送运动请求");
        send_goal(msg->pose);
    }

    void send_goal(const geometry_msgs::msg::Pose& target_pose)
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 不可用");
            return;
        }

        auto goal_msg = MoveEndToAbsPos::Goal();
        goal_msg.arm_id = "russ_group";
        goal_msg.pos = target_pose;

        RCLCPP_INFO(this->get_logger(), 
            "发送目标位姿: x=%.3f, y=%.3f, z=%.3f",
            goal_msg.pos.position.x,
            goal_msg.pos.position.y,
            goal_msg.pos.position.z);

        auto send_goal_options = rclcpp_action::Client<MoveEndToAbsPos>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CartesianActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&CartesianActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&CartesianActionClient::result_callback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleMoveEndToAbsPos::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "目标被接受");
    }

    void feedback_callback(
        GoalHandleMoveEndToAbsPos::SharedPtr,
        const std::shared_ptr<const MoveEndToAbsPos::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "运动进度: %.2f%%", feedback->completion_percentage);
    }

    void result_callback(const GoalHandleMoveEndToAbsPos::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "运动执行成功: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "运动被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "运动被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果代码");
                break;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}