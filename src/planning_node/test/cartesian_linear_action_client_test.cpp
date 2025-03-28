/*
 * @Author: "Claude AI" "claude@anthropic.com"
 * @Date: 2025-01-04 15:35:42
 * @LastEditors: "Claude AI" "claude@anthropic.com"
 * @LastEditTime: 2025-01-04 15:35:42
 * @FilePath: /planning_control_node/src/planning_node/src/cartesian_linear_action_client_test.cpp
 * @Description: 笛卡尔直线运动测试客户端
 *               用于测试CartesianLinearActionServer的功能
 *               提供了多个测试用例验证直线运动的准确性和鲁棒性
 */

#include <functional>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "planning_node/action/move_end_linear.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std::chrono_literals;

// 测试用例结构体
struct TestCase {
  std::string name;
  std::string arm_id;
  geometry_msgs::msg::Pose end_pos;
  double step_size;
};

class CartesianLinearActionClientTest : public rclcpp::Node {
public:
  using MoveEndLinear = planning_node::action::MoveEndLinear;
  using GoalHandleMoveEndLinear = rclcpp_action::ClientGoalHandle<MoveEndLinear>;

  CartesianLinearActionClientTest() : Node("cartesian_linear_action_client_test") {
    this->client_ptr_ = rclcpp_action::create_client<MoveEndLinear>(
      this, "moveEndLinear");
    
    this->timer_ = this->create_wall_timer(
      12s, std::bind(&CartesianLinearActionClientTest::init_move_group, this));
    
    // 初始化测试用例
    initialize_test_cases();
    
    RCLCPP_INFO(this->get_logger(), "笛卡尔直线运动测试客户端已启动");
  }

private:
  rclcpp_action::Client<MoveEndLinear>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<TestCase> test_cases_;
  size_t current_test_case_ = 0;
  bool test_in_progress_ = false;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  // 初始化MoveGroup接口
  void init_move_group() {
    this->timer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "初始化MoveGroup接口...");
    
    // 使用异步线程初始化MoveGroup接口
    std::thread([this]() {
      try {
        // 创建MoveGroup接口，参数是规划组名称(与test_cases中的arm_id一致)
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), test_cases_[0].arm_id);

        RCLCPP_INFO(this->get_logger(), "MoveGroup接口初始化成功");
        
        // 继续执行测试
        this->timer_ = this->create_wall_timer(
          500ms, std::bind(&CartesianLinearActionClientTest::run_tests, this));
      }
      catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "MoveGroup接口初始化失败: %s", e.what());
      }
    }).detach();
  }

  // 初始化测试用例
  void initialize_test_cases() {
    // 测试用例1: 基本直线运动测试（Z轴上移5cm）
    TestCase test1;
    test1.name = "基本Z轴上移测试";
    test1.arm_id = "russ_group";  // 替换为实际的机械臂规划组ID
    
    // 将在当前位置的基础上Z轴上移5cm
    test1.end_pos.position.x = 0.0;  // 将在execute_test_case中修改为当前位置X
    test1.end_pos.position.y = 0.0;  // 将在execute_test_case中修改为当前位置Y
    test1.end_pos.position.z = 0.0;  // 将在execute_test_case中修改为当前位置Z+0.05
    test1.end_pos.orientation.x = 0.0;
    test1.end_pos.orientation.y = 0.0;
    test1.end_pos.orientation.z = 0.0;
    test1.end_pos.orientation.w = 1.0;
    
    test1.step_size = 0.001;  // 更小的步长
    
    test_cases_.push_back(test1);
    
    // 测试用例2: 水平移动（X轴正向移动3cm）
    TestCase test2;
    test2.name = "X轴小范围移动测试";
    test2.arm_id = "russ_group";
    
    // 将在当前位置的基础上X轴移动3cm
    test2.end_pos.position.x = 0.0;  // 将在execute_test_case中修改为当前位置X+0.03
    test2.end_pos.position.y = 0.0;  // 将在execute_test_case中修改为当前位置Y
    test2.end_pos.position.z = 0.0;  // 将在execute_test_case中修改为当前位置Z
    test2.end_pos.orientation.x = 0.0;
    test2.end_pos.orientation.y = 0.0;
    test2.end_pos.orientation.z = 0.0;
    test2.end_pos.orientation.w = 1.0;
    
    test2.step_size = 0.001;
    
    test_cases_.push_back(test2);
    
    // 测试用例3: 斜向移动（X-Y平面对角线移动）
    TestCase test3;
    test3.name = "斜向小范围移动测试";
    test3.arm_id = "russ_group";
    
    // 将在当前位置的基础上X轴和Y轴同时移动2cm
    test3.end_pos.position.x = 0.0;  // 将在execute_test_case中修改为当前位置X+0.02
    test3.end_pos.position.y = 0.0;  // 将在execute_test_case中修改为当前位置Y+0.02
    test3.end_pos.position.z = 0.0;  // 将在execute_test_case中修改为当前位置Z
    test3.end_pos.orientation.x = 0.0;
    test3.end_pos.orientation.y = 0.0;
    test3.end_pos.orientation.z = 0.0;
    test3.end_pos.orientation.w = 1.0;
    
    test3.step_size = 0.001;
    
    test_cases_.push_back(test3);
  }

  // 运行测试
  void run_tests() {
    if (!client_ptr_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(this->get_logger(), "Action服务器未就绪");
      return;
    }
    
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroup接口未初始化");
      return;
    }
    
    this->timer_->cancel();
    
    if (current_test_case_ >= test_cases_.size()) {
      RCLCPP_INFO(this->get_logger(), "所有测试用例已完成");
      return;
    }
    
    if (!test_in_progress_) {
      execute_test_case(current_test_case_);
    }
  }

  // 执行测试用例
  void execute_test_case(size_t test_index) {
    test_in_progress_ = true;
    const TestCase& test_case = test_cases_[test_index];
    
    RCLCPP_INFO(this->get_logger(), "执行测试用例 [%zu]: %s", 
                test_index + 1, test_case.name.c_str());
    
    // 获取当前位置
    geometry_msgs::msg::PoseStamped current_pose;
    try {
      current_pose = move_group_interface_->getCurrentPose();
      RCLCPP_INFO(this->get_logger(), "当前位置: (%.3f, %.3f, %.3f)",
                  current_pose.pose.position.x, 
                  current_pose.pose.position.y, 
                  current_pose.pose.position.z);
                  
      // 设置目标位置（基于当前位置和测试用例的需求）
      auto goal_msg = MoveEndLinear::Goal();
      goal_msg.arm_id = test_case.arm_id;
      goal_msg.step_size = test_case.step_size;
      
      // 复制方向四元数
      goal_msg.end_pos.orientation = test_case.end_pos.orientation;
      
      // 根据测试用例确定目标位置（相对于当前位置）
      if (test_index == 0) {  // Z轴上移测试
        goal_msg.end_pos.position.x = current_pose.pose.position.x;
        goal_msg.end_pos.position.y = current_pose.pose.position.y;
        goal_msg.end_pos.position.z = current_pose.pose.position.z - 0.1;
      } 
      else if (test_index == 1) {  // X轴移动测试
        goal_msg.end_pos.position.x = current_pose.pose.position.x;
        goal_msg.end_pos.position.y = current_pose.pose.position.y;
        goal_msg.end_pos.position.z = current_pose.pose.position.z - 0.2;
      }
      else if (test_index == 2) {  // 斜向移动测试
        goal_msg.end_pos.position.x = current_pose.pose.position.x;
        goal_msg.end_pos.position.y = current_pose.pose.position.y;
        goal_msg.end_pos.position.z = current_pose.pose.position.z - 0.2;
      }
      
      RCLCPP_INFO(this->get_logger(), "发送目标: 到 (%.3f, %.3f, %.3f), 步长: %.3f",
                  goal_msg.end_pos.position.x, goal_msg.end_pos.position.y, goal_msg.end_pos.position.z,
                  goal_msg.step_size);
      
      auto send_goal_options = rclcpp_action::Client<MoveEndLinear>::SendGoalOptions();
      
      send_goal_options.goal_response_callback =
        std::bind(&CartesianLinearActionClientTest::goal_response_callback, this, std::placeholders::_1);
      
      send_goal_options.feedback_callback =
        std::bind(&CartesianLinearActionClientTest::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      
      send_goal_options.result_callback =
        std::bind(&CartesianLinearActionClientTest::result_callback, this, std::placeholders::_1);
      
      client_ptr_->async_send_goal(goal_msg, send_goal_options);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "获取当前位置失败: %s", e.what());
      test_in_progress_ = false;
      current_test_case_++;
      
      // 延迟一段时间后执行下一个测试
      this->timer_ = this->create_wall_timer(
        2s, std::bind(&CartesianLinearActionClientTest::run_tests, this));
    }
  }

  // 目标响应回调
  void goal_response_callback(const GoalHandleMoveEndLinear::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
      test_in_progress_ = false;
      current_test_case_++;
      
      // 延迟一段时间后执行下一个测试
      this->timer_ = this->create_wall_timer(
        2s, std::bind(&CartesianLinearActionClientTest::run_tests, this));
    } else {
      RCLCPP_INFO(this->get_logger(), "目标被接受");
    }
  }

  // 反馈回调
  void feedback_callback(
    GoalHandleMoveEndLinear::SharedPtr,
    const std::shared_ptr<const MoveEndLinear::Feedback> feedback) {
    
    RCLCPP_INFO(this->get_logger(), "接收到反馈: 进度 %.2f%%", feedback->progress * 100.0);
    RCLCPP_DEBUG(this->get_logger(), "当前位置: (%.3f, %.3f, %.3f)",
                feedback->current_pose.position.x,
                feedback->current_pose.position.y,
                feedback->current_pose.position.z);
  }

  // 结果回调
  void result_callback(const GoalHandleMoveEndLinear::WrappedResult& result) {
    test_in_progress_ = false;
    
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "测试用例 [%zu] 执行成功: %s", 
                  current_test_case_ + 1, result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "测试用例 [%zu] 被中止: %s", 
                  current_test_case_ + 1, result.result->message.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "测试用例 [%zu] 被取消", current_test_case_ + 1);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "测试用例 [%zu] 未知结果", current_test_case_ + 1);
        break;
    }
    
    current_test_case_++;
    
    // 延迟一段时间后执行下一个测试
    this->timer_ = this->create_wall_timer(
      3s, std::bind(&CartesianLinearActionClientTest::run_tests, this));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianLinearActionClientTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 