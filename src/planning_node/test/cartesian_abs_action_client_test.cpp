/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 17:16:42
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-08 17:27:53
 * @FilePath: /planning_control_node/src/planning_node/test/cartesian_abs_action_client_test.cpp
 * @Description: 笛卡尔绝对规划测试
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "planning_node/action/move_end_to_abs_pos.hpp"
#include <functional>

class CartesianAbsTestClient : public rclcpp::Node
{
public:
    using MoveEndToAbsPos = planning_node::action::MoveEndToAbsPos;
    using GoalHandleMoveAbs = rclcpp_action::ClientGoalHandle<MoveEndToAbsPos>;

    CartesianAbsTestClient() 
        : Node("cartesian_abs_test_client"), 
          current_test_case_(0),
          current_target_success_(false)
    {
        client_ = rclcpp_action::create_client<MoveEndToAbsPos>(
            this, "moveEndToAbsPos");

        // 定义测试用例
        test_cases_ = {
            {
                "测试用例1: 基本位置移动",
                {
                    create_target(0.4, 0.0, 0.5, "初始位置"),
                    create_target(0.5, 0.2, 0.3, "前方位置"),
                    create_target(0.4, -0.2, 0.3, "后方位置"),
                    create_target(0.4, 0.0, 0.5, "返回初始位置")

                    // {0.5, 0.0, 0.6},
                    // {0.5, 0.0, -0.1},
                    // {0.5, 0.3, 0.6},
                },
                {"初始位置", "前方位置", "后方位置", "返回初始位置"}  // 明确初始化描述列表
            },
            {
                "测试用例2: 避障测试",
                {
                    create_target(0.5, 0.0, 0.6, "避障可执行位置"),
                    create_target(0.5, 0.0, -0.1, "避障不可执行位置"),
                    create_target(0.5, 0.3, 0.6, "避障可执行位置"),
                },
                {"避障可执行位置", "避障不可执行位置", "避障可执行位置"}  // 明确初始化描述列表
            }
        };

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&CartesianAbsTestClient::run_next_test, this));
    }

private:
    rclcpp_action::Client<MoveEndToAbsPos>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    struct TestCase {
        std::string name;
        std::vector<geometry_msgs::msg::Pose> targets;
        std::vector<std::string> descriptions;
    };
    std::vector<TestCase> test_cases_;
    size_t current_test_case_;
    bool current_target_success_;

    // 创建目标位姿的辅助函数
    geometry_msgs::msg::Pose create_target(
        double x, double y, double z,
        [[maybe_unused]] const std::string& description)
    {
        geometry_msgs::msg::Pose target;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        target.orientation.w = 1.0;
        
        // test_cases_[test_cases_.size()-1].descriptions.push_back(description);
        return target;
    }

    void run_next_test()
    {
        timer_->cancel();

        if (current_test_case_ >= test_cases_.size()) {
            RCLCPP_INFO(this->get_logger(), "所有测试用例执行完成！");
            print_test_summary();
            rclcpp::shutdown();
            return;
        }

        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 未启动");
            rclcpp::shutdown();
            return;
        }

        const auto& test_case = test_cases_[current_test_case_];
        RCLCPP_INFO(this->get_logger(), "\n==================\n开始执行%s\n==================", 
            test_case.name.c_str());

        run_test_case(0);
    }

    void run_test_case(size_t target_index)
    {
        const auto& test_case = test_cases_[current_test_case_];
        if (target_index >= test_case.targets.size()) {
            print_test_case_summary();
            current_test_case_++;
            timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&CartesianAbsTestClient::run_next_test, this));
            return;
        }

        auto goal_msg = MoveEndToAbsPos::Goal();
        goal_msg.pos = test_case.targets[target_index];
        goal_msg.arm_id = "ur_group";

        RCLCPP_INFO(this->get_logger(), 
            "执行移动: %s (%.2f, %.2f, %.2f)",
            test_case.descriptions[target_index].c_str(),
            goal_msg.pos.position.x,
            goal_msg.pos.position.y,
            goal_msg.pos.position.z);

        current_target_success_ = false;

        auto send_goal_options = 
            rclcpp_action::Client<MoveEndToAbsPos>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&CartesianAbsTestClient::feedback_callback, this, 
                std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&CartesianAbsTestClient::result_callback, this,
                std::placeholders::_1, target_index);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedback_callback(
        GoalHandleMoveAbs::SharedPtr,
        const std::shared_ptr<const MoveEndToAbsPos::Feedback> feedback)
    {
        // const auto& test_case = test_cases_[current_test_case_];
        RCLCPP_INFO(this->get_logger(), 
            "测试用例 %ld - 当前位置: (%.2f, %.2f, %.2f), 进度: %.1f%%",
            current_test_case_ + 1,
            feedback->current_pose.position.x,
            feedback->current_pose.position.y,
            feedback->current_pose.position.z,
            feedback->completion_percentage);

        if (feedback->completion_percentage >= 100.0) {
            current_target_success_ = true;
        }
    }

    void result_callback(
        const GoalHandleMoveAbs::WrappedResult & result,
        size_t target_index)
    {
        const auto& test_case = test_cases_[current_test_case_];
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), 
                    "目标位置 %s 到达成功: %s",
                    test_case.descriptions[target_index].c_str(),
                    result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), 
                    "目标位置 %s 到达失败: %s",
                    test_case.descriptions[target_index].c_str(),
                    result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), 
                    "目标位置 %s 被取消: %s",
                    test_case.descriptions[target_index].c_str(),
                    result.result->message.c_str());
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知的结果码");
                break;
        }

        // 延迟一段时间后执行下一个目标
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this, target_index]() {
                timer_->cancel();
                run_test_case(target_index + 1);
            });
    }

    void print_test_case_summary()
    {
        const auto& test_case = test_cases_[current_test_case_];
        RCLCPP_INFO(this->get_logger(), "\n测试用例 %ld (%s) 执行完成", 
            current_test_case_ + 1, test_case.name.c_str());
        RCLCPP_INFO(this->get_logger(), "-------------------");
    }

    void print_test_summary()
    {
        RCLCPP_INFO(this->get_logger(), "\n=== 测试总结 ===");
        for (size_t i = 0; i < test_cases_.size(); ++i) {
            const auto& test_case = test_cases_[i];
            RCLCPP_INFO(this->get_logger(), "测试用例 %ld (%s): 完成",
                i + 1, test_case.name.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "==============");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianAbsTestClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 