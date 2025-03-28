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
#include <vector>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>

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
        (void)goal;
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
        
        // 增加规划尝试次数
        move_group->setNumPlanningAttempts(10);
        // 设置更长的规划时间以找到更优路径
        move_group->setPlanningTime(5.0);
        
        move_group->setMaxVelocityScalingFactor(0.3);
        move_group->setMaxAccelerationScalingFactor(0.3);
        
        // 设置路径约束，保持末端执行器姿态
        move_group->setGoalOrientationTolerance(0.1); // 角度容差（弧度）
    }

    // 计算轨迹的总关节运动量
    double calculateTotalJointMovement(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
    {
        double total_movement = 0.0;
        
        if (plan.trajectory_.joint_trajectory.points.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "轨迹点数量不足，无法计算关节运动量");
            return 0.0;
        }
        
        const auto& first_point = plan.trajectory_.joint_trajectory.points.front();
        const auto& last_point = plan.trajectory_.joint_trajectory.points.back();
        
        // 计算起点到终点的总关节运动量
        for (size_t j = 0; j < first_point.positions.size(); j++) {
            // 计算关节起点和终点的差值
            double joint_diff = std::abs(last_point.positions[j] - first_point.positions[j]);
            total_movement += joint_diff;
        }
        
        return total_movement;
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

            // 考虑使用笛卡尔路径规划而不是关节空间规划
            std::vector<geometry_msgs::msg::Pose> waypoints;
            geometry_msgs::msg::Pose target_pose = current_pose.pose;
            
            // 添加位置变化
            target_pose.position.x += goal->pos.position.x;
            target_pose.position.y += goal->pos.position.y;
            target_pose.position.z += goal->pos.position.z;
            
            // 考虑姿态变化 - 使用tf2四元数乘法来应用相对旋转
            if (goal->pos.orientation.x != 0.0 || 
                goal->pos.orientation.y != 0.0 || 
                goal->pos.orientation.z != 0.0 || 
                goal->pos.orientation.w != 0.0) {
                
                RCLCPP_INFO(this->get_logger(), "应用相对姿态变化");
                
                // 将当前姿态转换为tf2四元数
                tf2::Quaternion current_quat;
                tf2::fromMsg(current_pose.pose.orientation, current_quat);
                
                // 将目标相对姿态转换为tf2四元数
                tf2::Quaternion rel_quat;
                tf2::fromMsg(goal->pos.orientation, rel_quat);
                
                // 如果传入的是单位四元数(w=1)，可能表示不需要旋转
                if (std::abs(rel_quat.w()) != 1.0 || rel_quat.x() != 0.0 || rel_quat.y() != 0.0 || rel_quat.z() != 0.0) {
                    // 确保四元数已归一化
                    rel_quat.normalize();
                    
                    // 应用相对旋转 (current * relative = new)
                    tf2::Quaternion result_quat = current_quat * rel_quat;
                    result_quat.normalize();
                    
                    // 转换回ROS消息类型
                    target_pose.orientation = tf2::toMsg(result_quat);
                    
                    RCLCPP_INFO(this->get_logger(), "应用相对旋转，当前=[%.2f, %.2f, %.2f, %.2f], 相对=[%.2f, %.2f, %.2f, %.2f], 结果=[%.2f, %.2f, %.2f, %.2f]", 
                        current_quat.x(), current_quat.y(), current_quat.z(), current_quat.w(),
                        rel_quat.x(), rel_quat.y(), rel_quat.z(), rel_quat.w(),
                        result_quat.x(), result_quat.y(), result_quat.z(), result_quat.w());
                }
            }
            
            // 如果目标位置距离当前位置较近，使用笛卡尔路径规划
            double distance = std::sqrt(
                std::pow(goal->pos.position.x, 2) +
                std::pow(goal->pos.position.y, 2) +
                std::pow(goal->pos.position.z, 2));
                
            if (distance < 0.2) { // 如果距离小于20厘米
                RCLCPP_INFO(this->get_logger(), "使用笛卡尔路径规划，距离: %.3f米", distance);
                waypoints.push_back(target_pose);
                
                moveit_msgs::msg::RobotTrajectory trajectory;
                const double jump_threshold = 0.0;  // 禁用跳跃检测
                const double eef_step = 0.01;       // 1厘米分辨率
                
                double fraction = move_group_->computeCartesianPath(
                    waypoints, eef_step, jump_threshold, trajectory);
                    
                if (fraction > 0.9) {  // 成功计算至少90%的路径
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory;
                    
                    // 执行计算出的笛卡尔路径
                    auto execute_result = move_group_->execute(plan);
                    bool execute_success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
                    
                    if (execute_success) {
                        RCLCPP_INFO(this->get_logger(), "笛卡尔路径规划相对运动执行成功");
                        result->success = true;
                        result->message = "笛卡尔路径规划相对运动执行成功";
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "笛卡尔路径规划执行失败，错误代码: %d", execute_result.val);
                        result->success = false;
                        result->message = "笛卡尔路径规划执行失败，错误代码: " + std::to_string(execute_result.val);
                    }
                } else {
                    // 笛卡尔路径规划失败，回退到常规规划
                    RCLCPP_WARN(this->get_logger(), "笛卡尔路径规划失败(%.2f)，切换到常规规划", fraction);
                    move_group_->setPoseTarget(target_pose);
                    
                    // 实现多解策略，获取多个规划方案并选择最优解
                    RCLCPP_INFO(this->get_logger(), "开始多解规划");
                    
                    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
                    const int MAX_PLANS = 5; // 生成5个规划方案
                    
                    // 存储最佳方案的索引和其总关节运动量
                    int best_plan_index = -1;
                    double min_joint_movement = std::numeric_limits<double>::max();
                    
                    // 收集多个规划方案
                    for (int i = 0; i < MAX_PLANS; i++) {
                        moveit::planning_interface::MoveGroupInterface::Plan current_plan;
                        auto plan_result = move_group_->plan(current_plan);
                        bool plan_success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
                        
                        if (plan_success) {
                            plans.push_back(current_plan);
                            
                            // 使用独立函数计算关节总运动量
                            double total_movement = calculateTotalJointMovement(current_plan);
                            
                            RCLCPP_INFO(this->get_logger(), "规划方案 %d 的总关节运动量: %.4f", i, total_movement);
                            
                            // 检查是否为最佳方案
                            if (total_movement < min_joint_movement) {
                                min_joint_movement = total_movement;
                                best_plan_index = i;
                            }
                        }
                    }
                    
                    // 检查是否找到了有效的规划方案
                    if (best_plan_index >= 0) {
                        RCLCPP_INFO(this->get_logger(), "选择方案 %d 作为最优解，总运动量: %.4f", 
                                   best_plan_index, min_joint_movement);
                        
                        // 执行最优规划方案
                        auto& best_plan = plans[best_plan_index];
                        auto execute_result = move_group_->execute(best_plan);
                        bool execute_success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
                        
                        if (execute_success) {
                            RCLCPP_INFO(this->get_logger(), "RRT规划相对运动执行成功");
                            result->success = true;
                            result->message = "RRT规划相对运动执行成功";
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "RRT规划执行失败，错误代码: %d", execute_result.val);
                            result->success = false;
                            result->message = "RRT规划执行失败，错误代码: " + std::to_string(execute_result.val);
                        }
                    }
                }
            } else {
                // 对于较远距离，使用原有的规划方法
                move_group_->setPoseTarget(target_pose);
                
                // 执行规划
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                auto plan_result = move_group_->plan(plan);
                bool plan_success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (plan_success) {
                    RCLCPP_INFO(this->get_logger(), "RRT规划成功，目标位置: x=%.3f, y=%.3f, z=%.3f",
                               target_pose.position.x, target_pose.position.y, target_pose.position.z);
                    
                    // 开始执行运动
                    RCLCPP_INFO(this->get_logger(), "开始执行运动");
                    auto execute_result = move_group_->execute(plan);
                    bool execute_success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
                    
                    if (execute_success) {
                        RCLCPP_INFO(this->get_logger(), "RRT规划相对运动执行成功");
                        result->success = true;
                        result->message = "RRT规划相对运动执行成功";
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "RRT规划执行失败，错误代码: %d", execute_result.val);
                        result->success = false;
                        result->message = "RRT规划执行失败，错误代码: " + std::to_string(execute_result.val);
                    }
                } else {
                    RCLCPP_ERROR(this->get_logger(), "RRT规划失败，错误代码: %d", plan_result.val);
                    result->success = false;
                    result->message = "RRT规划失败，错误代码: " + std::to_string(plan_result.val);
                }
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