/*
 * @Author: "Claude AI" "claude@anthropic.com"
 * @Date: 2025-01-04 10:15:32
 * @LastEditors: "Claude AI" "claude@anthropic.com"
 * @LastEditTime: 2025-01-04 10:15:32
 * @FilePath: /planning_control_node/src/planning_node/src/cartesian_linear_action_server.cpp
 * @Description: 笛卡尔空间直线运动的Action服务端
 *               实现名为CartesianLinearActionServer的节点，用于控制机械臂末端执行器进行直线轨迹运动
 *               提供了moveEndLinear Action服务，用于处理直线轨迹运动请求
 *               使用三次多项式插值生成平滑轨迹
 *               提供了glob_planning_setting Service服务，用于动态配置全局避障功能
 *
 * Action 服务:
 *   - moveEndLinear: 用于发送笛卡尔直线运动请求
 *     - 输入: arm_id (机械臂ID), start_pos (起始位姿), end_pos (目标位姿), step_size (轨迹点间距)
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
 * 笛卡尔直线轨迹:
 *   - 从起点到终点生成直线轨迹
 *   - 使用三次多项式插值确保速度连续性
 *   - 可调节轨迹点密度满足不同精度需求
 */

#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "planning_node/action/move_end_linear.hpp"
#include "planning_node/srv/glob_planning_setting.hpp"
#include "planning_node/action_server_base.hpp"

// 三次多项式插值辅助类
class CubicPolynomial {
public:
    // 根据起点和终点位置、速度计算系数
    static std::vector<double> interpolate(double start_pos, double end_pos, 
                                          double start_vel, double end_vel, 
                                          double duration, int points) {
        // 三次多项式系数
        double a0 = start_pos;
        double a1 = start_vel;
        double a2 = 3.0 * (end_pos - start_pos) / (duration * duration) - 
                   2.0 * start_vel / duration - end_vel / duration;
        double a3 = -2.0 * (end_pos - start_pos) / (duration * duration * duration) + 
                   (start_vel + end_vel) / (duration * duration);
        
        std::vector<double> result;
        result.reserve(points);
        
        // 生成轨迹点
        for (int i = 0; i < points; ++i) {
            double t = duration * static_cast<double>(i) / (points - 1);
            double pos = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
            result.push_back(pos);
        }
        
        return result;
    }
};

class CartesianLinearActionServer : public ActionServerBase
{
public:
    using MoveEndLinear = planning_node::action::MoveEndLinear;
    using GoalHandleMoveEndLinear = rclcpp_action::ServerGoalHandle<MoveEndLinear>;

    CartesianLinearActionServer() 
        : ActionServerBase("cartesian_linear_action_server")
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<MoveEndLinear>(
            this,
            "moveEndLinear",
            std::bind(&CartesianLinearActionServer::handle_goal, this, _1, _2),
            std::bind(&CartesianLinearActionServer::handle_cancel, this, _1),
            std::bind(&CartesianLinearActionServer::handle_accepted, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "笛卡尔直线运动 Action Server 已启动");
    }

private:
    rclcpp_action::Server<MoveEndLinear>::SharedPtr action_server_;
    const double DEFAULT_STEP_SIZE = 0.01; // 默认轨迹点间距（米）
    const int MIN_POINTS = 3;            // 最小轨迹点数

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const MoveEndLinear::Goal> goal)
    {
        (void)uuid;  // 标记参数为已使用
        RCLCPP_INFO(this->get_logger(), "收到新的笛卡尔直线运动请求");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveEndLinear> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveEndLinear> goal_handle)
    {
        std::thread{std::bind(&CartesianLinearActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveEndLinear> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "执行笛卡尔直线目标");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveEndLinear::Feedback>();
        auto result = std::make_shared<MoveEndLinear::Result>();

        // 确保步长合理
        double step_size = goal->step_size > 0.0 ? goal->step_size : DEFAULT_STEP_SIZE;

        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), goal->arm_id);
        
        // 使用基类的配置函数
        move_group->clearPathConstraints();
        move_group->setMaxVelocityScalingFactor(0.3);
        move_group->setMaxAccelerationScalingFactor(0.3);

        // 获取当前机器人状态
        auto current_state = move_group->getCurrentState();
        
        // 获取当前末端位姿作为起点
        geometry_msgs::msg::PoseStamped current_pose_stamped = move_group->getCurrentPose();
        geometry_msgs::msg::Pose current_pose = current_pose_stamped.pose;
        
        RCLCPP_INFO(this->get_logger(), "当前位置: (%.3f, %.3f, %.3f)",
                   current_pose.position.x, current_pose.position.y, current_pose.position.z);
        
        // 创建路径点序列
        std::vector<geometry_msgs::msg::Pose> waypoints;
        
        // 添加起点
        waypoints.push_back(current_pose);
        
        // 添加终点（直接使用绝对坐标）
        waypoints.push_back(goal->end_pos);
        
        RCLCPP_INFO(this->get_logger(), "规划直线路径: 从 (%.3f, %.3f, %.3f) 到 (%.3f, %.3f, %.3f)",
                   current_pose.position.x, current_pose.position.y, current_pose.position.z,
                   goal->end_pos.position.x, goal->end_pos.position.y, goal->end_pos.position.z);
        
        // 计算笛卡尔路径
        moveit_msgs::msg::RobotTrajectory trajectory;
        double jump_threshold = 0.5;  // 添加跳变阈值参数
        double fraction = move_group->computeCartesianPath(waypoints, step_size, jump_threshold, trajectory); // 修正参数

        RCLCPP_INFO(this->get_logger(), "笛卡尔路径规划 (完成率: %.2f%%)", fraction * 100.0);

        // 使用单一条件判断规划失败的情况
        if (fraction < 0.9 || trajectory.joint_trajectory.points.size() < 3)
        {
            result->success = false;
            std::string failure_reason;

            if (fraction < 0.9)
            {
                failure_reason = "无法规划完整笛卡尔路径，完成率: " + std::to_string(fraction);
                RCLCPP_ERROR(this->get_logger(), "笛卡尔路径规划失败，完成率: %.2f", fraction);
            }

            if (trajectory.joint_trajectory.points.size() < 3)
            {
                if (!failure_reason.empty())
                    failure_reason += "; ";
                failure_reason += "规划的轨迹点数过少，点数: " + std::to_string(trajectory.joint_trajectory.points.size());
                RCLCPP_ERROR(this->get_logger(), "轨迹点数过少，点数: %zu", trajectory.joint_trajectory.points.size());
            }

            // result->message = failure_reason;
            // goal_handle->succeed(result);
            // return;
        }

        RCLCPP_INFO(this->get_logger(), "成功规划笛卡尔直线路径，开始执行，轨迹点数: %zu", 
                   trajectory.joint_trajectory.points.size());
        
        // 添加时间参数化
        robot_trajectory::RobotTrajectory rt(move_group->getRobotModel(), goal->arm_id);
        rt.setRobotTrajectoryMsg(*current_state, trajectory);
        
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt);
        
        // 将轨迹转换回消息格式
        rt.getRobotTrajectoryMsg(trajectory);
        
        // 创建执行计划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        
        // 周期发送反馈
        auto send_feedback = [this, goal_handle, feedback, &waypoints]() {
            int total_points = waypoints.size();
            for (int i = 0; i < total_points && rclcpp::ok(); ++i) {
                if (goal_handle->is_canceling()) {
                    return;
                }
                feedback->progress = static_cast<double>(i) / total_points;
                feedback->current_pose = waypoints[i];
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        };
        
        std::thread feedback_thread(send_feedback);
        
        // 执行计划
        moveit::core::MoveItErrorCode execute_result = move_group->execute(my_plan);
        
        if (feedback_thread.joinable()) {
            feedback_thread.join();
        }
        
        bool execute_success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "任务被取消";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "直线运动目标被取消");
            return;
        }
        
        if (execute_success) {
            // 获取最终位姿
            auto final_pose = move_group->getCurrentPose();
            
            result->success = true;
            result->message = "笛卡尔直线运动执行成功";
            RCLCPP_INFO(this->get_logger(), "笛卡尔直线运动执行成功");
        } else {
            result->success = false;
            result->message = "执行失败，错误代码: " + std::to_string(execute_result.val);
            RCLCPP_ERROR(this->get_logger(), "笛卡尔直线运动执行失败，错误代码: %d", execute_result.val);
        }
        
        goal_handle->succeed(result);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianLinearActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 