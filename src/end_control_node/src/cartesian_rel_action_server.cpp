/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-02 15:55:20
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-03 12:21:32
 * @FilePath: /end_control_node/src/end_control_node/src/cartesian_rel_action_server.cpp
 * @Description: 笛卡尔空间相对位置运动的Action服务端
 *               实现名为RelativeMotionServer的节点，用于控制机械臂末端执行器进行相对位置运动
 *               提供了moveEndToRelPos Action服务，用于处理相对运动请求
 *
 * Action 服务端:
 *   - moveEndToRelPos: 用于发送相对位置运动请求
 *     - 输入: arm_id (机械臂ID), pos (目标位姿)
 *     - 输出: success (执行结果), message (结果信息)
 */

#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "end_control_node/action/move_end_to_rel_pos.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RelativeMotionServer : public rclcpp::Node
{
public:
    using MoveEndToRelPos = end_control_node::action::MoveEndToRelPos;
    using GoalHandleMoveEndToRelPos = rclcpp_action::ServerGoalHandle<MoveEndToRelPos>;

    RelativeMotionServer()
        : Node("cartesian_rel_action_server_node")
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<MoveEndToRelPos>(
            this,
            "moveEndToRelPos",
            std::bind(&RelativeMotionServer::handle_goal, this, _1, _2),
            std::bind(&RelativeMotionServer::handle_cancel, this, _1),
            std::bind(&RelativeMotionServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "相对运动 Action Server 已启动");
    }

private:
    rclcpp_action::Server<MoveEndToRelPos>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveEndToRelPos::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到新的相对运动请求");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveEndToRelPos> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveEndToRelPos> goal_handle)
    {
        // 使用共享指针保持线程引用
        auto execute_thread = std::make_shared<std::thread>(
            std::bind(&RelativeMotionServer::execute, this, goal_handle));
        execute_thread->detach();
    }

    void configure_move_group(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
    {
        move_group->setPlannerId("BiTRRT");
        move_group->clearPathConstraints();
        
        move_group->setMaxVelocityScalingFactor(0.3);
        move_group->setMaxAccelerationScalingFactor(0.3);
    }

    void execute(const std::shared_ptr<GoalHandleMoveEndToRelPos> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "执行相对运动目标");
        
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<MoveEndToRelPos::Result>();
        
        try {
            // 初始化MoveGroup
            if (!move_group_) {
                move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), goal->arm_id);
            }

            configure_move_group(move_group_);

            // 获取当前位姿
            geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

            // 计算目标位姿（当前位姿 + 相对位移）
            geometry_msgs::msg::Pose target_pose = current_pose.pose;
            target_pose.position.x += goal->pos.position.x;
            target_pose.position.y += goal->pos.position.y;
            target_pose.position.z += goal->pos.position.z;
            
            // 设置目标位姿
            move_group_->setPoseTarget(target_pose);
            
            // 执行规划
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto plan_result = move_group_->plan(plan);
            bool plan_success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (plan_success) {
                RCLCPP_INFO(this->get_logger(), "规划成功，目标位置: x=%.3f, y=%.3f, z=%.3f",
                           target_pose.position.x, target_pose.position.y, target_pose.position.z);
                
                // 开始执行运动
                RCLCPP_INFO(this->get_logger(), "开始执行运动");
                auto execute_result = move_group_->execute(plan);
                bool execute_success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (execute_success) {
                    RCLCPP_INFO(this->get_logger(), "相对运动执行成功");
                    result->success = true;
                    result->message = "相对运动执行成功";
                } else {
                    RCLCPP_ERROR(this->get_logger(), "执行失败，错误代码: %d", execute_result.val);
                    result->success = false;
                    result->message = "执行失败，错误代码: " + std::to_string(execute_result.val);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "规划失败，错误代码: %d", plan_result.val);
                result->success = false;
                result->message = "规划失败，错误代码: " + std::to_string(plan_result.val);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "执行过程出错: %s", e.what());
            result->success = false;
            result->message = "执行过程出错: " + std::string(e.what());
        }
        
        // 检查是否被取消
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "任务被取消";
            RCLCPP_INFO(this->get_logger(), "发送取消结果到客户端");
            goal_handle->canceled(result);
            return;
        }
        
        // 发送结果
        RCLCPP_INFO(this->get_logger(), "发送成功结果到客户端");
        goal_handle->succeed(result);
        
        // 确保结果被发送
        RCLCPP_INFO(this->get_logger(), "结果已发送，等待客户端接收");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelativeMotionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}