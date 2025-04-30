/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 17:15:52
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-06 14:17:09
 * @FilePath: /planning_control_node/src/planning_node/src/waypoint_action_client.cpp
 * @Description:
 *   航点动作客户端测试节点，提供以下功能：
 *   适用于ur_group图像跟踪测试
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
        goal_msg.arm_id = "ur_group";

        // 创建测试航点序列
        std::vector<planning_node::msg::Waypoint> waypoints;

        // man_link原点坐标: 0.066; 0.79525; 0.79344
        
        // 初始点
        planning_node::msg::Waypoint init_point;
        init_point.waypoint_type = planning_node::msg::Waypoint::TYPE_INIT;
        init_point.waypoint_mode = planning_node::msg::Waypoint::MODE_PICK;
        init_point.waypoint_pose.position.x = 0.5;
        init_point.waypoint_pose.position.y = 0.2;
        init_point.waypoint_pose.position.z = 0.5;
        init_point.waypoint_pose.orientation.x = 0.0;
        init_point.waypoint_pose.orientation.y = 0.0;
        init_point.waypoint_pose.orientation.z = 0.7071068;
        init_point.waypoint_pose.orientation.w = 0.7071068;
        init_point.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_STOP;
        waypoints.push_back(init_point);

        // 0.81642;
        // 0.19129;
        // 0.15983

        // 工作点1 - 取料位置
        planning_node::msg::Waypoint work_point1;
        work_point1.waypoint_type = planning_node::msg::Waypoint::TYPE_WORK;
        work_point1.waypoint_mode = planning_node::msg::Waypoint::MODE_PICK;
        work_point1.waypoint_pose.position.x = 0.6;
        work_point1.waypoint_pose.position.y = 0.2;
        work_point1.waypoint_pose.position.z = 0.5;
        work_point1.waypoint_pose.orientation.x = 0.0;
        work_point1.waypoint_pose.orientation.y = 0.0;
        work_point1.waypoint_pose.orientation.z = 0.7071068;
        work_point1.waypoint_pose.orientation.w = 0.7071068;
        work_point1.waypoint_plan_mode = planning_node::msg::Waypoint::PLAN_SMOOTH;
        work_point1.action_type = planning_node::msg::Waypoint::ACTION_END_CONTROL;
        work_point1.end_effector_type = planning_node::msg::Waypoint::END_PROBE;
        work_point1.end_effector_action = 1.0;  // 开启探针
        waypoints.push_back(work_point1);

        // 结束点
        planning_node::msg::Waypoint end_point;
        end_point.waypoint_type = planning_node::msg::Waypoint::TYPE_END;
        end_point.waypoint_mode = planning_node::msg::Waypoint::MODE_PICK;
        end_point.waypoint_pose.position.x = 0.5;
        end_point.waypoint_pose.position.y = 0.2;
        end_point.waypoint_pose.position.z = 0.6;
        end_point.waypoint_pose.orientation.x = 0.0;
        end_point.waypoint_pose.orientation.y = 0.0;
        end_point.waypoint_pose.orientation.z = 0.7071068;
        end_point.waypoint_pose.orientation.w = 0.7071068;
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