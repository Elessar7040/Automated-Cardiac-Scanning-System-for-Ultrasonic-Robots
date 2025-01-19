/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-02 15:36:42
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-03 14:11:37
 * @FilePath: /planning_control_node/src/planning_node/src/cartesian_abs_action_server.cpp
 * @Description: 笛卡尔空间绝对位置运动的Action服务端
 *               实现名为CartesianActionServer的节点，用于控制机械臂末端执行器进行绝对位置运动
 *               提供了moveEndToAbsPos Action服务，用于处理绝对运动请求
 *               提供了glob_planning_setting Service服务，用于动态配置全局避障功能
 *
 * Action 服务:
 *   - moveEndToAbsPos: 用于发送绝对位置运动请求
 *     - 输入: arm_id (机械臂ID), pos (目标位姿)
 *     - 输出: success (执行结果), message (结果信息)
 *
 * Service 服务:
 *   - glob_planning_setting: 用于配置全局避障功能
 *     - 输入: obstacle_avoidance_enabled (避障开关), planning_mode (规划模式)
 *     - 输出: success (配置结果), message (结果信息)
 *     - 规划模式:
 *       - 0: 普通模式 (默认，中等速度和精度)
 *       - 1: 安全模式 (低速，高精度)
 *       - 2: 快速模式 (高速，低精度)
 *       - 3: 精确模式 (低速，超高精度)
 *
 * 避障功能:
 *   - 开启时: 使用完整的避障规划，考虑所有场景中的障碍物
 *   - 关闭时: 临时移除所有障碍物，直接规划运动路径
 *   - 重新开启时: 恢复之前保存的场景障碍物
 */

#include <functional>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include "planning_node/action/move_end_to_abs_pos.hpp"
#include "planning_node/srv/glob_planning_setting.hpp"
#include "planning_node/action_server_base.hpp"

class CartesianActionServer : public ActionServerBase
{
public:
    using MoveEndToAbsPos = planning_node::action::MoveEndToAbsPos;
    using GoalHandleMoveEndToAbsPos = rclcpp_action::ServerGoalHandle<MoveEndToAbsPos>;

    CartesianActionServer() 
        : ActionServerBase("cartesian_abs_action_server")
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<MoveEndToAbsPos>(
            this,
            "moveEndToAbsPos",
            std::bind(&CartesianActionServer::handle_goal, this, _1, _2),
            std::bind(&CartesianActionServer::handle_cancel, this, _1),
            std::bind(&CartesianActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "笛卡尔运动 Action Server 已启动");
    }

private:
    rclcpp_action::Server<MoveEndToAbsPos>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MoveEndToAbsPos::Goal> goal)
    {
        (void)uuid;  // 标记参数为已使用
        (void)goal;  // 标记参数为已使用
        RCLCPP_INFO(this->get_logger(), "收到新的目标位姿请求");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveEndToAbsPos> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveEndToAbsPos> goal_handle)
    {
        std::thread{std::bind(&CartesianActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveEndToAbsPos> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "执行目标");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveEndToAbsPos::Feedback>();
        auto result = std::make_shared<MoveEndToAbsPos::Result>();

        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), goal->arm_id);
        
        // 使用基类的配置函数
        configure_move_group(move_group);

        // 设置目标位姿
        move_group->setPoseTarget(goal->pos);

        // 规划和执行
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            // 执行运动
            move_group->execute(my_plan);

            // 获取最终位姿
            auto final_pose = move_group->getCurrentPose();
            
            result->success = true;
            result->message = "运动执行成功";
        } else {
            result->success = false;
            result->message = "规划失败";
        }

        goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}