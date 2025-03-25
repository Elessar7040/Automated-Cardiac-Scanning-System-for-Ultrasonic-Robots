/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-02 16:11:14
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-06 14:21:25
 * @FilePath: /end_control_node/src/end_control_node/src/cartesian_rel_action_client.cpp
 * @Description: 笛卡尔空间相对位置运动的Action客户端。
 *               实现机械臂末端执行器移动到当前位置的相对位置。
 * 
 * Action 客户端:
 *   - moveEndToRelPos: 用于发送相对位置运动请求
 *     - 输入: arm_id (机械臂ID), pos (目标位姿)
 *     - 输出: success (执行结果), message (结果信息)
 */
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "end_control_node/action/move_end_to_rel_pos.hpp"

class RelativeMotionClient : public rclcpp::Node
{
public:
    using MoveEndToRelPos = end_control_node::action::MoveEndToRelPos;
    using GoalHandleMoveEndToRelPos = rclcpp_action::ClientGoalHandle<MoveEndToRelPos>;

    RelativeMotionClient() : Node("relative_motion_client")
    {
        client_ptr_ = rclcpp_action::create_client<MoveEndToRelPos>(
            this, "moveEndToRelPos");

        // 创建参数
        this->declare_parameter("arm_id", "russ_group");
        this->declare_parameter("rel_x", 0.1);
        this->declare_parameter("rel_y", 0.0);
        this->declare_parameter("rel_z", 0.1);
        this->declare_parameter("rel_qw", 1.0);
        this->declare_parameter("rel_qx", 0.0);
        this->declare_parameter("rel_qy", 0.0);
        this->declare_parameter("rel_qz", 0.0);

        RCLCPP_INFO(this->get_logger(), "相对运动客户端已启动");
        send_goal();
    }

private:
    rclcpp_action::Client<MoveEndToRelPos>::SharedPtr client_ptr_;

    void send_goal()
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 不可用");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = MoveEndToRelPos::Goal();
        
        // 从参数获取值
        goal_msg.arm_id = this->get_parameter("arm_id").as_string();
        goal_msg.pos.position.x = this->get_parameter("rel_x").as_double();
        goal_msg.pos.position.y = this->get_parameter("rel_y").as_double();
        goal_msg.pos.position.z = this->get_parameter("rel_z").as_double();
        goal_msg.pos.orientation.w = this->get_parameter("rel_qw").as_double();
        goal_msg.pos.orientation.x = this->get_parameter("rel_qx").as_double();
        goal_msg.pos.orientation.y = this->get_parameter("rel_qy").as_double();
        goal_msg.pos.orientation.z = this->get_parameter("rel_qz").as_double();

        RCLCPP_INFO(this->get_logger(), 
            "发送相对运动请求: x=%.3f, y=%.3f, z=%.3f",
            goal_msg.pos.position.x,
            goal_msg.pos.position.y,
            goal_msg.pos.position.z);

        auto send_goal_options = rclcpp_action::Client<MoveEndToRelPos>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&RelativeMotionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&RelativeMotionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&RelativeMotionClient::result_callback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleMoveEndToRelPos::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "目标被接受");
    }

    void feedback_callback(
        GoalHandleMoveEndToRelPos::SharedPtr,
        const std::shared_ptr<const MoveEndToRelPos::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "当前完成度: %.2f%%", feedback->completion_percentage);
    }

    void result_callback(const GoalHandleMoveEndToRelPos::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "目标成功完成: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "目标被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "目标被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果代码");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelativeMotionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}