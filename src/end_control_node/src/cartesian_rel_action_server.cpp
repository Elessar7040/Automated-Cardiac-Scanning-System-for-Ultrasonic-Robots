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
        : Node("cartesian_rel_action_server")
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
        std::thread{std::bind(&RelativeMotionServer::execute, this, goal_handle)}.detach();
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
        auto feedback = std::make_shared<MoveEndToRelPos::Feedback>();
        auto result = std::make_shared<MoveEndToRelPos::Result>();

        // 创建MoveGroup接口
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), goal->arm_id);
        
        // 配置MoveGroup
        configure_move_group(move_group);

        // 获取当前末端位姿
        geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();
        
        // 计算目标位姿（当前位姿 + 相对位姿）
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.pose.position.x + goal->pos.position.x;
        target_pose.position.y = current_pose.pose.position.y + goal->pos.position.y;
        target_pose.position.z = current_pose.pose.position.z + goal->pos.position.z;

        // 处理姿态（四元数相乘）
        tf2::Quaternion q_current, q_relative, q_target;
        tf2::fromMsg(current_pose.pose.orientation, q_current);
        tf2::fromMsg(goal->pos.orientation, q_relative);
        q_target = q_current * q_relative;
        q_target.normalize();
        target_pose.orientation = tf2::toMsg(q_target);

        // 设置目标位姿
        move_group->setPoseTarget(target_pose);

        // 规划和执行
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool plan_success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (plan_success) {
            RCLCPP_INFO(this->get_logger(), 
                "规划成功，目标位置: x=%.3f, y=%.3f, z=%.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

            // 执行运动并检查执行结果
            moveit::core::MoveItErrorCode execute_result = move_group->execute(my_plan);
            bool execute_success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);

            if (execute_success) {
                result->success = true;
                result->message = "相对运动执行成功";
            } else {
                result->success = false;
                result->message = "执行失败，错误代码: " + std::to_string(execute_result.val);
                RCLCPP_ERROR(this->get_logger(), "运动执行失败，错误代码: %d", execute_result.val);
            }
        } else {
            result->success = false;
            result->message = "相对运动规划失败";
            RCLCPP_ERROR(this->get_logger(), "路径规划失败");
        }
        
        // 检查是否被取消
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "任务被取消";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "目标被取消");
            return;
        }
        
        goal_handle->succeed(result);
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