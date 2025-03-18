/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 17:15:52
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-06 14:17:09
 * @FilePath: /planning_control_node/src/planning_node/src/waypoint_action_client.cpp
 * @Description: 
 *   航点动作客户端测试节点，提供以下功能：
 *   1. 发送航点序列运动请求
 *   2. 航点类型支持：
 *      - TYPE_INIT: 初始点
 *      - TYPE_WORK: 工作点
 *      - TYPE_END: 结束点
 *   
 *   3. 运动规划模式：
 *      - PLAN_STOP: 停止规划（用于起始点）
 *      - PLAN_SMOOTH: 平滑规划（标准模式）
 *      - PLAN_FORCE: 强制规划（特殊情况）
 *   
 *   4. 末端执行器控制：
 *      - END_SUCTION: 吸盘控制
 *        * action = 1.0: 开启（等待2秒）
 *        * action = 0.0: 关闭（等待3秒）
 *   
 *   5. 实时反馈处理：
 *      - 完成百分比
 *      - 当前执行航点
 *      - 执行状态信息
 *   
 *   测试航点序列：
 *   1. 初始点 (-0.4, 0.0, 0.5)
 *   2. 取料点 (-0.5, 0.2, 0.3) 开启吸盘
 *   3. 放料点 (-0.4, -0.2, 0.3) 关闭吸盘
 *   4. 结束点 (-0.4, 0.0, 0.5)
 *   
 *   使用方法：
 *   1. 启动节点：ros2 run planning_node waypoint_action_client
 *   2. 依赖服务：
 *      - waypointMotion action server
 *      - end_control service
 *   
 *   注意事项：
 *   1. 确保航点序列格式正确（初始点-工作点-结束点）
 *   2. 航点位置在机械臂工作空间内
 *   3. 支持任务取消和错误处理
 *   4. 末端执行器动作会自动处理等待时间
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "planning_node/action/waypoint_motion.hpp"
#include "planning_node/waypoint_types.hpp"
#include <functional>


class WaypointActionClient : public rclcpp::Node
{
public:
    using WaypointMotion = planning_node::action::WaypointMotion;
    using GoalHandleWaypointMotion = rclcpp_action::ClientGoalHandle<WaypointMotion>;

    WaypointActionClient() : Node("waypoint_action_client")
    {
        client_ = rclcpp_action::create_client<WaypointMotion>(
            this, "waypointMotion");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&WaypointActionClient::send_goal, this));
    }

    void send_goal()
    {
        timer_->cancel(); // 只发送一次

        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = WaypointMotion::Goal();
        // goal_msg.arm_id = robot_.attr("robot_group").cast<std::string>();
        goal_msg.arm_id = "russ_group";

        // 创建测试航点序列
        std::vector<planning_node::msg::Waypoint> waypoints;

        // man_link原点坐标: 0.066; 0.79525; 0.79344
        // -0.70236; 0.003093; 3.1671e-06; 0.71182
        
        // 初始点
        planning_node::msg::Waypoint init_point;
        init_point.waypoint_type = planning_node::msg::Waypoint::TYPE_INIT;
        init_point.waypoint_mode = planning_node::msg::Waypoint::MODE_PICK;
        init_point.waypoint_pose.position.x = 0.1;
        init_point.waypoint_pose.position.y = 0.7;
        init_point.waypoint_pose.position.z = 1.2;
        // init_point.waypoint_pose.orientation.w = 1.0;
        init_point.waypoint_pose.orientation.x = 0.7071068;
        init_point.waypoint_pose.orientation.y = 0.0;
        init_point.waypoint_pose.orientation.z = 0.0;
        init_point.waypoint_pose.orientation.w = -0.7071068;
        init_point.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_STOP;
        waypoints.push_back(init_point);

        // 工作点0 - 原点
        planning_node::msg::Waypoint work_point0;
        work_point0.waypoint_type = planning_node::msg::Waypoint::TYPE_WORK;
        work_point0.waypoint_mode = planning_node::msg::Waypoint::MODE_PICK;
        work_point0.waypoint_pose.position.x = 0.07;
        work_point0.waypoint_pose.position.y = 0.8;
        work_point0.waypoint_pose.position.z = 0.9;
        work_point0.waypoint_pose.orientation.x = 0.7071068;
        work_point0.waypoint_pose.orientation.y = 0.0;
        work_point0.waypoint_pose.orientation.z = 0.0;
        work_point0.waypoint_pose.orientation.w = -0.7071068;
        work_point0.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_SMOOTH;
        work_point0.action_type = planning_node::msg::Waypoint::ACTION_END_CONTROL;
        work_point0.end_effector_type = planning_node::msg::Waypoint::END_SUCTION;
        work_point0.end_effector_action = 1.0; // 开启吸盘
        waypoints.push_back(work_point0);

        // 工作点1 - 上点
        planning_node::msg::Waypoint work_point1;
        work_point1.waypoint_type = planning_node::msg::Waypoint::TYPE_WORK;
        work_point1.waypoint_mode = planning_node::msg::Waypoint::MODE_PICK;
        work_point1.waypoint_pose.position.x = 0.09;
        work_point1.waypoint_pose.position.y = 0.8;
        work_point1.waypoint_pose.position.z = 0.82;
        work_point1.waypoint_pose.orientation.x = 0.7071068;
        work_point1.waypoint_pose.orientation.y = 0.0;
        work_point1.waypoint_pose.orientation.z = 0.0;
        work_point1.waypoint_pose.orientation.w = -0.7071068;
        work_point1.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_SMOOTH;
        work_point1.action_type = planning_node::msg::Waypoint::ACTION_END_CONTROL;
        work_point1.end_effector_type = planning_node::msg::Waypoint::END_SUCTION;
        work_point1.end_effector_action = 1.0;  // 开启吸盘
        waypoints.push_back(work_point1);

        // 工作点2 - 右点
        planning_node::msg::Waypoint work_point2;
        work_point2.waypoint_type = planning_node::msg::Waypoint::TYPE_WORK;
        work_point2.waypoint_mode = planning_node::msg::Waypoint::MODE_UNLOAD;
        work_point2.waypoint_pose.position.x = 0.06;
        work_point2.waypoint_pose.position.y = 0.8;
        work_point2.waypoint_pose.position.z = 0.80;
        work_point2.waypoint_pose.orientation.x = 0.7071068;
        work_point2.waypoint_pose.orientation.y = 0.0;
        work_point2.waypoint_pose.orientation.z = 0.0;
        work_point2.waypoint_pose.orientation.w = -0.7071068;
        work_point2.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_FORCE;
        work_point2.action_type = planning_node::msg::Waypoint::ACTION_END_CONTROL;
        work_point2.end_effector_type = planning_node::msg::Waypoint::END_SUCTION;
        work_point2.end_effector_action = 0.0;  // 关闭吸盘
        waypoints.push_back(work_point2);

        // 工作点3 - 下点
        planning_node::msg::Waypoint work_point3;
        work_point3.waypoint_type = planning_node::msg::Waypoint::TYPE_WORK;
        work_point3.waypoint_mode = planning_node::msg::Waypoint::MODE_UNLOAD;
        work_point3.waypoint_pose.position.x = 0.04;
        work_point3.waypoint_pose.position.y = 0.8;
        work_point3.waypoint_pose.position.z = 0.80;
        work_point3.waypoint_pose.orientation.x = 0.7071068;
        work_point3.waypoint_pose.orientation.y = 0.0;
        work_point3.waypoint_pose.orientation.z = 0.0;
        work_point3.waypoint_pose.orientation.w = -0.7071068;
        work_point3.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_FORCE;
        work_point3.action_type = planning_node::msg::Waypoint::ACTION_END_CONTROL;
        work_point3.end_effector_type = planning_node::msg::Waypoint::END_SUCTION;
        work_point3.end_effector_action = 0.0; // 关闭吸盘
        waypoints.push_back(work_point3);

        // 工作点4 - 左点
        planning_node::msg::Waypoint work_point4;
        work_point4.waypoint_type = planning_node::msg::Waypoint::TYPE_WORK;
        work_point4.waypoint_mode = planning_node::msg::Waypoint::MODE_UNLOAD;
        work_point4.waypoint_pose.position.x = 0.06;
        work_point4.waypoint_pose.position.y = 0.8;
        work_point4.waypoint_pose.position.z = 0.79;
        work_point4.waypoint_pose.orientation.x = 0.7071068;
        work_point4.waypoint_pose.orientation.y = 0.0;
        work_point4.waypoint_pose.orientation.z = 0.0;
        work_point4.waypoint_pose.orientation.w = -0.7071068;
        work_point4.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_FORCE;
        work_point4.action_type = planning_node::msg::Waypoint::ACTION_END_CONTROL;
        work_point4.end_effector_type = planning_node::msg::Waypoint::END_SUCTION;
        work_point4.end_effector_action = 0.0; // 关闭吸盘
        waypoints.push_back(work_point4);

        // 结束点
        planning_node::msg::Waypoint end_point;
        end_point.waypoint_type = planning_node::msg::Waypoint::TYPE_END;
        end_point.waypoint_mode = planning_node::msg::Waypoint::MODE_PICK;
        end_point.waypoint_pose.position.x = 0.5;
        end_point.waypoint_pose.position.y = 0.7;
        end_point.waypoint_pose.position.z = 1.1;
        end_point.waypoint_pose.orientation.w = 1.0;
        end_point.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_STOP;
        waypoints.push_back(end_point);

        goal_msg.waypoints = waypoints;

        RCLCPP_INFO(this->get_logger(), "发送航点运动请求");

        auto send_goal_options = 
            rclcpp_action::Client<WaypointMotion>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&WaypointActionClient::feedback_callback, this, 
                std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&WaypointActionClient::result_callback, this, 
                std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<WaypointMotion>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void feedback_callback(
        GoalHandleWaypointMotion::SharedPtr,
        const std::shared_ptr<const WaypointMotion::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
            "当前执行进度: %.1f%%, 航点: %d, 状态: %s",
            feedback->completion_percentage,
            feedback->current_waypoint + 1,
            feedback->current_state.c_str());
    }

    void result_callback(const GoalHandleWaypointMotion::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "任务成功完成: %s", 
                    result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "任务失败: %s", 
                    result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "任务被取消: %s", 
                    result.result->message.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的结果码");
                break;
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 