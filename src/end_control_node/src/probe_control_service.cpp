/*
 * @Author: "your_name" "your_email"
 * @Date: 2025-01-xx xx:xx:xx
 * @LastEditors: "your_name" "your_email"
 * @LastEditTime: 2025-01-xx xx:xx:xx
 * @FilePath: /end_control_node/src/end_control_node/src/probe_control_service.cpp
 * @Description: 探针控制服务
 *               实现名为ProbeControlService的节点，用于控制探针执行视觉伺服
 *               提供了probeControl服务，用于处理探针控制请求
 * 
 * ************************************
 *              未实际使用
 * ************************************
 */

#include <memory>
#include <chrono>
#include <thread>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "end_control_node/srv/probe_control.hpp"
#include "end_control_node/action/move_end_to_rel_pos.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <mutex>
#include <condition_variable>

using namespace std::chrono_literals;

class ProbeControlService : public rclcpp::Node
{
public:
    using MoveEndToRelPos = end_control_node::action::MoveEndToRelPos;
    using GoalHandleMoveEndToRelPos = rclcpp_action::ClientGoalHandle<MoveEndToRelPos>;

    ProbeControlService()
    : Node("probe_control_service")
    {
        // 创建探针控制服务
        probe_service_ = this->create_service<end_control_node::srv::ProbeControl>(
            "probe_control",
            std::bind(&ProbeControlService::handle_probe_request, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // 创建相对运动Action客户端
        rel_motion_client_ = rclcpp_action::create_client<MoveEndToRelPos>(
            this, "moveEndToRelPos");
        
        // 订阅目标中心点话题
        target_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/yolo/target_center", 10,
            std::bind(&ProbeControlService::target_callback, this, std::placeholders::_1));
        
        // 初始化参数
        image_width_ = this->declare_parameter("image_width", 1264.0);
        image_height_ = this->declare_parameter("image_height", 880.0);
        pixel_to_meter_factor_ = this->declare_parameter("pixel_to_meter_factor", 0.0001);
        position_tolerance_pixels_ = this->declare_parameter("position_tolerance_pixels", 10.0);
        max_visual_servo_attempts_ = this->declare_parameter("max_visual_servo_attempts", 3);
        
        // 初始化目标中心点
        latest_target_center_.x = image_width_ / 2.0;
        latest_target_center_.y = image_height_ / 2.0;
        latest_target_center_.z = 0.0;
        
        has_target_ = false;
        
        RCLCPP_INFO(this->get_logger(), "探针控制服务已启动");
    }

private:
    rclcpp::Service<end_control_node::srv::ProbeControl>::SharedPtr probe_service_;
    rclcpp_action::Client<MoveEndToRelPos>::SharedPtr rel_motion_client_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscription_;
    
    geometry_msgs::msg::Point latest_target_center_;
    bool has_target_;
    
    double image_width_;
    double image_height_;
    double pixel_to_meter_factor_;
    double position_tolerance_pixels_;
    int max_visual_servo_attempts_;
    
    std::mutex action_mutex_;
    std::condition_variable action_cv_;
    bool action_completed_ = false;
    bool action_success_ = false;
    std::string action_message_;
    
    // 处理探针控制请求
    void handle_probe_request(
        const std::shared_ptr<end_control_node::srv::ProbeControl::Request> request,
        std::shared_ptr<end_control_node::srv::ProbeControl::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到探针控制请求");
        
        try {
            // 执行视觉伺服
            bool success = perform_visual_servo(request->pose);
            
            response->success = success;
            if (success) {
                response->message = "探针控制成功";
            } else {
                response->message = "探针控制失败";
            }
            
        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("探针控制异常: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "探针控制异常: %s", e.what());
        }
    }
    
    // 目标中心点回调
    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        latest_target_center_ = *msg;
        has_target_ = true;
    }
    
    // 执行视觉伺服
    bool perform_visual_servo(const geometry_msgs::msg::Pose& pose)
    {
        RCLCPP_INFO(this->get_logger(), "执行探针控制，目标位置: [%f, %f, %f]",
            pose.position.x, pose.position.y, pose.position.z);
        
        // 检查是否有目标点
        if (!has_target_) {
            RCLCPP_ERROR(this->get_logger(), "未检测到目标点");
            return false;
        }
        
        // 图像中心点
        double center_x = image_width_ / 2.0;
        double center_y = image_height_ / 2.0;
        
        // 阶段1: 粗定位 - 使用较大的移动步长和较宽松的容差
        double coarse_pixel_to_meter_factor = pixel_to_meter_factor_ * 1.5;
        double coarse_position_tolerance = position_tolerance_pixels_ * 3.0;
        
        RCLCPP_INFO(this->get_logger(), "阶段1: 粗定位开始");
        if (!servo_phase(center_x, center_y, 3, coarse_position_tolerance, coarse_pixel_to_meter_factor)) {
            RCLCPP_ERROR(this->get_logger(), "粗定位阶段失败");
            // return false;
        }
        
        // 阶段2: 精定位 - 使用较小的移动步长和严格的容差
        RCLCPP_INFO(this->get_logger(), "阶段2: 精定位开始");
        if (!servo_phase(center_x, center_y, max_visual_servo_attempts_ - 3, 
                        position_tolerance_pixels_, pixel_to_meter_factor_ * 0.8)) {
            RCLCPP_WARN(this->get_logger(), "精定位阶段未达到最佳位置，但继续执行");
            // 即使精定位不完美，我们也继续执行
        }
        
        // 阶段3: 执行探针操作
        // RCLCPP_INFO(this->get_logger(), "阶段3: 执行探针操作");
        std::this_thread::sleep_for(std::chrono::seconds(4));
        
        RCLCPP_INFO(this->get_logger(), "探针控制完成");
        return true;
    }
    
    // 视觉伺服阶段
    bool servo_phase(double center_x, double center_y, int max_attempts, 
                    double tolerance, double scale_factor)
    {
        int attempts = 0;
        
        // 首先检查 Action 服务器是否可用
        RCLCPP_INFO(this->get_logger(), "检查相对运动 Action 服务器是否可用...");
        if (!rel_motion_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "相对运动 Action 服务器不可用，无法执行视觉伺服");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "相对运动 Action 服务器已就绪");
        
        while (attempts < max_attempts) {
            attempts++;
            
            // 计算目标点与图像中心的偏差
            double error_x = latest_target_center_.x - center_x;
            double error_y = latest_target_center_.y - center_y;
            
            RCLCPP_INFO(this->get_logger(), "伺服尝试 %d: 偏差(像素) x=%.2f, y=%.2f, 移动(米) y=%.4f, z=%.4f",
                       attempts, error_x, error_y, 
                       -error_x * scale_factor, -error_y * scale_factor);
            
            // 检查是否已经达到容差范围
            if (std::abs(error_x) < tolerance && std::abs(error_y) < tolerance) {
                RCLCPP_INFO(this->get_logger(), "目标已在容差范围内，偏差(像素): x=%.2f, y=%.2f", 
                           error_x, error_y);
                // return true;
            }
            
            // 创建目标消息
            auto goal_msg = MoveEndToRelPos::Goal();
            goal_msg.arm_id = "russ_group";  // 使用正确的机械臂ID
            
            // 正确设置Pose类型
            goal_msg.pos.position.x = 0.0;  // 不移动x方向
            goal_msg.pos.position.y = -error_x * scale_factor;  // 将x像素误差转换为y方向移动
            goal_msg.pos.position.z = -error_y * scale_factor;  // 将y像素误差转换为z方向移动
            
            // 设置方向四元数为单位四元数（不旋转）
            goal_msg.pos.orientation.x = 0.0;
            goal_msg.pos.orientation.y = 0.0;
            goal_msg.pos.orientation.z = 0.0;
            goal_msg.pos.orientation.w = 1.0;
            
            RCLCPP_INFO(this->get_logger(), "发送相对运动目标: y=%.4f, z=%.4f", 
                       goal_msg.pos.position.y, goal_msg.pos.position.z);
            
            // 添加重试机制
            const int max_action_retries = 3;
            int action_retry = 0;
            bool action_success = false;
            
            while (action_retry < max_action_retries && !action_success) {
                action_retry++;
                if (action_retry > 1) {
                    RCLCPP_WARN(this->get_logger(), "重试相对运动 (尝试 %d/%d)", 
                               action_retry, max_action_retries);
                }
                
                try {
                    // 重置Action状态
                    {
                        std::lock_guard<std::mutex> lock(action_mutex_);
                        action_completed_ = false;
                        action_success_ = false;
                        action_message_ = "";
                    }
                    
                    // 设置发送目标的回调
                    auto send_goal_options = rclcpp_action::Client<MoveEndToRelPos>::SendGoalOptions();
                    
                    send_goal_options.goal_response_callback = 
                        [this](const GoalHandleMoveEndToRelPos::SharedPtr & goal_handle) {
                            if (!goal_handle) {
                                std::lock_guard<std::mutex> lock(action_mutex_);
                                action_completed_ = true;
                                action_success_ = false;
                                action_message_ = "目标被拒绝";
                                action_cv_.notify_one();
                            } else {
                                RCLCPP_INFO(this->get_logger(), "目标被接受");
                            }
                        };
                        
                    send_goal_options.feedback_callback =
                        [this](GoalHandleMoveEndToRelPos::SharedPtr,
                               const std::shared_ptr<const MoveEndToRelPos::Feedback> feedback) {
                            RCLCPP_INFO(this->get_logger(), "收到反馈");
                        };
                        
                    send_goal_options.result_callback =
                        [this](const GoalHandleMoveEndToRelPos::WrappedResult & result) {
                            std::lock_guard<std::mutex> lock(action_mutex_);
                            action_completed_ = true;
                            
                            switch (result.code) {
                            case rclcpp_action::ResultCode::SUCCEEDED:
                                action_success_ = result.result->success;
                                action_message_ = result.result->message;
                                RCLCPP_INFO(this->get_logger(), "目标成功完成: %s", 
                                           action_success_ ? "成功" : "失败");
                                break;
                            case rclcpp_action::ResultCode::ABORTED:
                                action_success_ = false;
                                action_message_ = "目标被中止";
                                RCLCPP_ERROR(this->get_logger(), "目标被中止");
                                break;
                            case rclcpp_action::ResultCode::CANCELED:
                                action_success_ = false;
                                action_message_ = "目标被取消";
                                RCLCPP_ERROR(this->get_logger(), "目标被取消");
                                break;
                            default:
                                action_success_ = false;
                                action_message_ = "未知结果代码";
                                RCLCPP_ERROR(this->get_logger(), "未知结果代码");
                                break;
                            }
                            
                            action_cv_.notify_one();
                        };
                    
                    // 发送目标
                    rel_motion_client_->async_send_goal(goal_msg, send_goal_options);
                    
                    // 等待结果，增加超时时间
                    {
                        std::unique_lock<std::mutex> lock(action_mutex_);
                        if (!action_cv_.wait_for(lock, std::chrono::seconds(60),
                                              [this]() { return action_completed_; })) {
                            RCLCPP_ERROR(this->get_logger(), "等待Action结果超时");
                            continue;  // 尝试下一次重试
                        }
                        
                        if (!action_success_) {
                            RCLCPP_ERROR(this->get_logger(), "相对运动执行失败: %s", action_message_.c_str());
                            continue;  // 尝试下一次重试
                        }
                        
                        // 如果到达这里，说明动作成功完成
                        action_success = true;
                    }
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "执行相对运动时发生异常: %s", e.what());
                    // 继续尝试下一次重试
                }
            }
            
            if (!action_success) {
                RCLCPP_ERROR(this->get_logger(), "相对运动执行失败，已达到最大重试次数");
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "相对运动执行成功，继续视觉伺服");
            
            // 等待新的目标检测结果
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        RCLCPP_WARN(this->get_logger(), "视觉伺服未能完全对准目标，已达到最大尝试次数");
        return false;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ProbeControlService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 