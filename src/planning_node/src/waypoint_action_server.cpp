/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 14:23:49
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-07 14:02:21
 * @FilePath: /planning_control_node/src/planning_node/src/waypoint_action_server.cpp
 * @Description: 
 *   航点动作服务器节点，提供以下功能：
 *   1. 接收并执行航点序列运动请求
 *   2. 支持多种运动规划模式（停止、平滑、强制）
 *   3. 支持末端执行器控制（吸盘开关）
 *   4. 提供实时反馈（当前航点、完成进度、执行状态）
 *   5. 可配置避障功能
 *   
 *   使用方法：
 *   1. 启动节点：ros2 run planning_node waypoint_action_server
 *   2. 服务接口：waypointMotion (planning_node/action/WaypointMotion)
 *   3. 依赖服务：end_control (end_control_node/srv/EndControl)
 *   
 *   注意事项：
 *   1. 航点序列必须包含初始点、工作点和结束点
 *   2. 末端执行器动作会自动处理等待时间
 *   3. 支持任务取消和错误恢复
 */
#include "planning_node/action_server_base.hpp"
#include "planning_node/action/waypoint_motion.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <functional>
#include "end_control_node/srv/end_control.hpp"

class WaypointActionServer : public ActionServerBase
{
public:
    using WaypointMotion = planning_node::action::WaypointMotion;
    using GoalHandleWaypointMotion = rclcpp_action::ServerGoalHandle<WaypointMotion>;

    WaypointActionServer()
        : ActionServerBase("waypoint_action_server")
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<WaypointMotion>(
            this,
            "waypointMotion",
            std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
            std::bind(&WaypointActionServer::handle_cancel, this, _1),
            std::bind(&WaypointActionServer::handle_accepted, this, _1));
            
        RCLCPP_INFO(this->get_logger(), "航点动作服务器已启动");

        // 创建末端控制服务客户端
        end_control_client_ = this->create_client<end_control_node::srv::EndControl>(
            "end_control");
    }

private:
    rclcpp_action::Server<WaypointMotion>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Client<end_control_node::srv::EndControl>::SharedPtr end_control_client_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const WaypointMotion::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "收到新的航点运动请求");
        (void)uuid;
        
        // 检查航点列表是否为空
        if (goal->waypoints.empty()) {
            RCLCPP_ERROR(this->get_logger(), "航点列表为空");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 验证航点序列的合法性
        if (!validate_waypoints(goal->waypoints)) {
            RCLCPP_ERROR(this->get_logger(), "航点序列不合法");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWaypointMotion> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleWaypointMotion> goal_handle)
    {
        std::thread{std::bind(&WaypointActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    bool validate_waypoints(const std::vector<planning_node::msg::Waypoint>& waypoints)
    {
        if (waypoints.empty()) return false;

        // 检查起点和终点类型
        if (waypoints.front().waypoint_type != planning_node::msg::Waypoint::TYPE_INIT ||
            waypoints.back().waypoint_type != planning_node::msg::Waypoint::TYPE_END) {
            RCLCPP_ERROR(this->get_logger(), "航点序列必须以初始点开始，结束点结束");
            return false;
        }

        // 检查中间点
        for (size_t i = 1; i < waypoints.size() - 1; ++i) {
            if (waypoints[i].waypoint_type != planning_node::msg::Waypoint::TYPE_WORK) {
                RCLCPP_ERROR(this->get_logger(), "中间航点必须是工作点");
                return false;
            }
        }

        return true;
    }

    void execute(const std::shared_ptr<GoalHandleWaypointMotion> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "开始执行航点运动");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<WaypointMotion::Feedback>();
        auto result = std::make_shared<WaypointMotion::Result>();
        
        try {
            // 初始化MoveGroup
            if (!move_group_) {
                move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), goal->arm_id);
                move_group_->setPlannerId("BiTRRT");
            }

            // 配置MoveGroup参数
            configure_move_group(move_group_);

            // 执行每个航点
            for (size_t i = 0; i < goal->waypoints.size(); ++i) {
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->message = "任务被取消";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "航点运动被取消");
                    return;
                }

                const auto& waypoint = goal->waypoints[i];
                
                // 更新反馈
                feedback->current_waypoint = i;
                feedback->completion_percentage = 
                    static_cast<float>(i) / goal->waypoints.size() * 100.0;
                feedback->current_state = "正在执行航点 " + std::to_string(i + 1);
                goal_handle->publish_feedback(feedback);

                // 执行航点动作
                if (!execute_waypoint(waypoint)) {
                    result->success = false;
                    result->message = "执行航点 " + std::to_string(i + 1) + " 失败";
                    goal_handle->abort(result);
                    return;
                }
            }

            // 成功完成所有航点
            result->success = true;
            result->message = "所有航点执行完成";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "航点运动执行完成");

        } catch (const std::exception& e) {
            result->success = false;
            result->message = "执行过程出错: " + std::string(e.what());
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "航点运动执行失败: %s", e.what());
        }
    }

    bool move_to_target(const geometry_msgs::msg::Pose& target_pose)
    {
        try {
            // 设置目标位姿
            move_group_->setPoseTarget(target_pose);

            // 执行规划和移动
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            
            // 多次尝试规划
            bool success = false;
            int max_attempts = 5;
            double position_tolerance = 0.01;  // 初始位置容差
            double orientation_tolerance = 0.1;  // 初始方向容差
            
            for (int attempt = 0; attempt < max_attempts && !success; ++attempt) {
                RCLCPP_INFO(this->get_logger(), "规划尝试 %d/%d", attempt + 1, max_attempts);
                
                // 设置当前尝试的容差
                move_group_->setGoalPositionTolerance(position_tolerance);
                move_group_->setGoalOrientationTolerance(orientation_tolerance);
                
                auto plan_result = move_group_->plan(plan);
                bool plan_success = (plan_result == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (plan_success) {
                    RCLCPP_INFO(this->get_logger(), "规划成功，执行移动");
                    auto execute_result = move_group_->execute(plan);
                    success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
                    
                    if (!success) {
                        RCLCPP_ERROR(this->get_logger(), "执行失败，错误代码: %d", execute_result.val);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "规划失败，增加容差后重试...");
                    // 每次失败后增加容差
                    position_tolerance *= 1.2;
                    orientation_tolerance *= 1.2;
                }
            }

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "在%d次尝试后规划或执行失败", max_attempts);
            }

            return success;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "移动失败: %s", e.what());
            return false;
        }
    }

    void setPathConstraints(const geometry_msgs::msg::Pose& target_pose)
    {
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.name = "keep_orientation";

        // 添加方向约束
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = move_group_->getEndEffectorLink();
        ocm.header.frame_id = move_group_->getPlanningFrame();
        ocm.orientation = target_pose.orientation;
        ocm.absolute_x_axis_tolerance = 0.3;
        ocm.absolute_y_axis_tolerance = 0.3;
        ocm.absolute_z_axis_tolerance = 0.3;
        ocm.weight = 0.5;

        path_constraints.orientation_constraints.push_back(ocm);
        move_group_->setPathConstraints(path_constraints);
    }

    bool execute_waypoint(const planning_node::msg::Waypoint& waypoint)
    {
        try {
            // 执行运动
            if (!move_to_target(waypoint.waypoint_pose)) {
                return false;
            }

            // 处理末端执行器动作
            if (waypoint.action_type == planning_node::msg::Waypoint::ACTION_END_CONTROL) {
                if (waypoint.end_effector_type == planning_node::msg::Waypoint::END_SUCTION) {
                    auto request = std::make_shared<end_control_node::srv::EndControl::Request>();
                    request->device_type = request->TYPE_SUCTION;
                    request->device_id = 1;
                    request->action = waypoint.end_effector_action;
                    request->pose = waypoint.waypoint_pose;

                    // 使用同步调用
                    auto result = end_control_client_->async_send_request(request).get();
                    if (!result || !result->success) {
                        RCLCPP_ERROR(this->get_logger(), "末端控制失败: %s", 
                            result ? result->message.c_str() : "调用失败");
                        return false;
                    }
                }
            }

            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "执行航点失败: %s", e.what());
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 