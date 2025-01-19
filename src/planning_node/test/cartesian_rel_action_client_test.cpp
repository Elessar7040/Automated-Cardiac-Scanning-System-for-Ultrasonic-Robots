/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-07 17:12:20
 * @LastEditors: “feiyang_hong” “feiyang.hong@infinityrobot.cn”
 * @LastEditTime: 2025-01-08 17:28:00
 * @FilePath: /planning_control_node/src/planning_node/test/cartesian_rel_action_client_test.cpp
 * @Description: 笛卡尔相对规划测试
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "planning_node/action/move_end_to_rel_pos.hpp"
#include <functional>

class CartesianRelTestClient : public rclcpp::Node
{
public:
    using MoveEndToRelPos = planning_node::action::MoveEndToRelPos;
    using GoalHandleMoveRel = rclcpp_action::ClientGoalHandle<MoveEndToRelPos>;

    CartesianRelTestClient() 
        : Node("cartesian_rel_test_client"), 
          current_test_case_(0),
          current_target_success_(false)
    {
        client_ = rclcpp_action::create_client<MoveEndToRelPos>(
            this, "moveEndToRelPos");

        // 初始化第一个测试用例
        test_cases_.push_back({"测试用例1: 基本相对运动", {}, {}});
        test_cases_.back().targets.push_back(create_target(0.1, 0.0, 0.0, "向前移动10cm"));
        test_cases_.back().targets.push_back(create_target(0.0, 0.1, 0.0, "向右移动10cm"));
        test_cases_.back().targets.push_back(create_target(0.0, 0.0, 0.1, "向上移动10cm"));
        test_cases_.back().targets.push_back(create_target(-0.1, -0.1, -0.1, "返回起点"));

        // 初始化第二个测试用例
        test_cases_.push_back({"测试用例2: 复合相对运动", {}, {}});
        test_cases_.back().targets.push_back(create_target(0.1, 0.1, 0.0, "斜向前右移动"));
        test_cases_.back().targets.push_back(create_target(-0.1, 0.1, 0.1, "斜向后右上移动"));
        test_cases_.back().targets.push_back(create_target(0.0, -0.2, -0.1, "返回起点"));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&CartesianRelTestClient::run_next_test, this));
    }

private:
    rclcpp_action::Client<MoveEndToRelPos>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    struct TestCase {
        std::string name;
        std::vector<geometry_msgs::msg::Pose> targets;
        std::vector<std::string> descriptions;
    };
    std::vector<TestCase> test_cases_;
    size_t current_test_case_;
    bool current_target_success_;

    // 创建相对运动目标的辅助函数
    geometry_msgs::msg::Pose create_target(
        double dx, double dy, double dz, 
        const std::string& description)
    {
        geometry_msgs::msg::Pose target;
        target.position.x = dx;  // 相对位移
        target.position.y = dy;
        target.position.z = dz;
        target.orientation.w = 1.0;
        
        test_cases_.back().descriptions.push_back(description);
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
                std::bind(&CartesianRelTestClient::run_next_test, this));
            return;
        }

        auto goal_msg = MoveEndToRelPos::Goal();
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
            rclcpp_action::Client<MoveEndToRelPos>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&CartesianRelTestClient::feedback_callback, this, 
                std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&CartesianRelTestClient::result_callback, this,
                std::placeholders::_1, target_index);

        client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedback_callback(
        GoalHandleMoveRel::SharedPtr,
        const std::shared_ptr<const MoveEndToRelPos::Feedback> feedback)
    {
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
        const GoalHandleMoveRel::WrappedResult & result,
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
    auto node = std::make_shared<CartesianRelTestClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}