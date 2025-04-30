/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-06 11:24:23
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-07 15:32:05
 * @FilePath: /planning_control_node/src/end_control_node/src/end_control_service.cpp 
 * @Description: 
 *   末端控制服务节点，提供以下功能：
 *   1. 提供末端执行器控制服务接口
 *   2. 支持多种末端执行器类型（吸盘、夹爪）
 *   3. 提供详细的执行状态反馈
 *   
 *   使用方法：
 *   1. 启动节点：ros2 run end_control_node end_control_service
 *   2. 服务接口：end_control (end_control_node/srv/EndControl)
 *   3. 服务调用示例：
 *      ros2 service call /end_control end_control_node/srv/EndControl "{
 *        device_type: 1,    # 1:吸盘, 2:夹爪
 *        device_id: 1,      # 设备ID
 *        action: 1.0        # 1.0:开启, 0.0:关闭
 *      }"
 *   
 *   注意事项：
 *   1. 服务调用是同步的，会等待动作完成后返回
 *   2. 确保硬件连接正常后再启动服务
 *   3. 支持错误处理和状态反馈
 */
#include <rclcpp/rclcpp.hpp>
#include "end_control_node/srv/end_control.hpp"
#include <memory>
#include <thread>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>

// 使用end_control_node自己的action定义
#include "end_control_node/action/move_end_to_rel_pos.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

// 添加服务客户端头文件
#include "std_srvs/srv/trigger.hpp"

// 添加Bool消息类型
#include <std_msgs/msg/bool.hpp>

class EndControlService : public rclcpp::Node
{
public:
    using MoveEndToRelPos = end_control_node::action::MoveEndToRelPos;
    using GoalHandleMoveEndToRelPos = rclcpp_action::ClientGoalHandle<MoveEndToRelPos>;
    
    EndControlService()
    : Node("end_control_service")
    {
        // 创建回调组
        callback_group_clients_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
            
        callback_group_services_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 创建Action客户端并指定回调组
        rel_motion_client_ = rclcpp_action::create_client<MoveEndToRelPos>(
            this,
            "moveEndToRelPos",
            callback_group_clients_);

        // 创建服务服务器并指定回调组
        service_ = create_service<end_control_node::srv::EndControl>(
            "end_control", 
            std::bind(&EndControlService::handle_request, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_services_);

        // 订阅YOLO目标中心点话题
        target_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/yolo/target_center", 1,
            std::bind(&EndControlService::target_callback, this, std::placeholders::_1));
            
        // 订阅YOLO处理后的图像
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/yolo/detected_image", 1,
            std::bind(&EndControlService::image_callback, this, std::placeholders::_1));
            
        // 创建YOLO服务客户端
        yolo_client_ = this->create_client<std_srvs::srv::Trigger>(
            "start_yolo_detection",
            rmw_qos_profile_services_default,
            callback_group_clients_);
            
        // 创建YOLO控制发布者
        yolo_control_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/yolo/start_detection",
            1);
            
        // 订阅YOLO处理状态
        yolo_status_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/yolo/processing_status",
            1,
            std::bind(&EndControlService::yolo_status_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "末端控制服务已启动");
    }

private:
    rclcpp::Service<end_control_node::srv::EndControl>::SharedPtr service_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp_action::Client<MoveEndToRelPos>::SharedPtr rel_motion_client_;
    
    // 存储最新的目标中心点
    geometry_msgs::msg::Point latest_target_center_;
    bool has_target_ = false;
    
    // 存储图像尺寸
    // 用于russ_group
    // int image_width_ = 1264;  // 默认值
    // int image_height_ = 880; // 默认值

    // 用于ur_group图像跟踪测试
    int image_width_ = 640; // 默认值
    int image_height_ = 480; // 默认值
    bool has_image_ = false;

    // 视觉伺服参数
    // 像素到米的转换因子
    // const double pixel_to_meter_factor_ = 0.0002;  // 用于russ_group
    const double pixel_to_meter_factor_ = 0.0005;  // 用于ur_group图像跟踪测试
    const int max_visual_servo_attempts_ = 3;      // 最大视觉伺服尝试次数
    const double position_tolerance_pixels_ = 10.0; // 位置容差（像素）
    
    // 用于跟踪异步操作状态的成员变量
    std::mutex action_mutex_;
    std::condition_variable action_cv_;
    bool action_completed_ = false;
    bool action_success_ = false;
    std::string action_message_;
    
    // 添加回调组成员变量
    rclcpp::CallbackGroup::SharedPtr callback_group_clients_;
    rclcpp::CallbackGroup::SharedPtr callback_group_services_;
    
    // 添加YOLO服务客户端
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr yolo_client_;
    
    // 添加YOLO控制发布者
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr yolo_control_publisher_;
    
    // 添加YOLO状态标志
    std::atomic<bool> yolo_processing_{false};
    
    // 添加YOLO状态订阅者
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr yolo_status_subscriber_;
    
    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        latest_target_center_ = *msg;
        has_target_ = true;
        RCLCPP_DEBUG(this->get_logger(), "收到目标中心点: x=%.2f, y=%.2f", 
                    latest_target_center_.x, latest_target_center_.y);
    }
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_width_ = msg->width;
        image_height_ = msg->height;
        has_image_ = true;
        RCLCPP_DEBUG(this->get_logger(), "收到图像，尺寸: %dx%d", image_width_, image_height_);
    }

    void yolo_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        yolo_processing_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "YOLO处理状态更新: %s", 
            yolo_processing_ ? "进行中" : "已完成");
    }

    // 目标响应回调
    void goal_response_callback(const GoalHandleMoveEndToRelPos::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            std::lock_guard<std::mutex> lock(action_mutex_);
            action_completed_ = true;
            action_success_ = false;
            action_message_ = "目标被拒绝";
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
            action_cv_.notify_one();
        } else {
            RCLCPP_INFO(this->get_logger(), "目标被接受");
        }
    }

    // 反馈回调
    void feedback_callback(
        GoalHandleMoveEndToRelPos::SharedPtr,
        const std::shared_ptr<const MoveEndToRelPos::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "当前完成度: %.2f%%", feedback->completion_percentage);
    }

    // 结果回调
    void result_callback(const GoalHandleMoveEndToRelPos::WrappedResult & result)
    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        action_completed_ = true;
        
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            action_success_ = result.result->success;
            action_message_ = result.result->message;
            RCLCPP_INFO(this->get_logger(), "目标成功完成: %s", result.result->message.c_str());
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
            RCLCPP_ERROR(this->get_logger(), "未知结果代码: %d", static_cast<int>(result.code));
            break;
        }
        
        RCLCPP_INFO(this->get_logger(), "结果回调被调用，通知等待线程");
        action_cv_.notify_one();
    }

    bool control_suction(uint8_t suction_id, bool enable)
    {
        try {
            RCLCPP_INFO(this->get_logger(), 
                "控制吸盘 %d: %s", 
                suction_id, 
                enable ? "开启" : "关闭");

            // 使用 sleep 替代 rclcpp::sleep_for
            if (enable) {
                RCLCPP_INFO(this->get_logger(), "吸盘开启，停留2秒");
                std::this_thread::sleep_for(std::chrono::seconds(2));
            } else {
                RCLCPP_INFO(this->get_logger(), "吸盘关闭，停留3秒");
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
            
            RCLCPP_INFO(this->get_logger(), "吸盘控制完成");
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "控制吸盘失败: %s", e.what());
            return false;
        }
    }

    void handle_request(
        const std::shared_ptr<end_control_node::srv::EndControl::Request> request,
        std::shared_ptr<end_control_node::srv::EndControl::Response> response)
    {
        try {
            switch (request->device_type) {
                case end_control_node::srv::EndControl::Request::TYPE_SUCTION:
                    response->success = control_suction(
                        request->device_id,
                        request->action > 0.5);
                    break;

                case end_control_node::srv::EndControl::Request::TYPE_PROBE:
                    {  // 添加花括号创建新的作用域
                        bool probe_success = control_probe(request->pose);
                        // bool probe_success = control_probe_2(request->pose);
                        if (!probe_success)
                        {
                            RCLCPP_WARN(this->get_logger(), "探针未能完全对准目标");
                            // 继续执行，但记录警告
                        }
                        response->success = probe_success;
                    }  // 作用域结束
                    break;

                default:
                    response->success = false;
                    response->message = "未知的设备类型";
                    return;
            }

            if (response->success) {
                response->message = "操作成功";
            }

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("操作失败: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "服务处理异常: %s", e.what());
        }
    }
    
    bool control_probe(const geometry_msgs::msg::Pose& initial_pose)
    {
        try {
            RCLCPP_INFO(this->get_logger(), "执行探针控制，目标位置: [%f, %f, %f]",
                initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
                
            // 发布启动YOLO检测的消息
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            yolo_control_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "已发送YOLO检测启动信号");
            
            // 等待一小段时间确保YOLO节点已启动
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
            // 执行视觉伺服
            bool servo_success = perform_visual_servo();
            if (!servo_success) {
                RCLCPP_ERROR(this->get_logger(), "视觉伺服失败");
                return false;
            }
            
            // 检查是否有目标点
            if (!has_target_) {
                RCLCPP_ERROR(this->get_logger(), "未检测到目标点，继续执行视觉伺服");
                // return false;
            }
            
            // 模拟探针动作
            RCLCPP_INFO(this->get_logger(), "探针到达目标位置，执行探针动作");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // 发送停止YOLO检测的消息
            msg.data = false;
            yolo_control_publisher_->publish(msg);
            
            RCLCPP_INFO(this->get_logger(), "探针控制完成");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "探针控制失败: %s", e.what());
            return false;
        }
    }
    bool control_probe_2(const geometry_msgs::msg::Pose &initial_pose)
    {
        // 使用 initial_pose 参数
        RCLCPP_INFO(this->get_logger(), "执行探针控制，目标位置: [%f, %f, %f]",
                    initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);

        // 检查是否有目标点
        if (!has_target_)
        {
            RCLCPP_ERROR(this->get_logger(), "未检测到目标点，继续执行视觉伺服");
            // return false;
        }

        // 执行视觉伺服
        bool servo_success = perform_visual_servo();
        if (!servo_success)
        {
            RCLCPP_ERROR(this->get_logger(), "视觉伺服失败");
            return false;
        }

        // 模拟探针动作
        RCLCPP_INFO(this->get_logger(), "探针到达目标位置，执行探针动作");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        RCLCPP_INFO(this->get_logger(), "探针控制完成");
        return true;
    }
    
    bool perform_visual_servo()
    {
        RCLCPP_INFO(this->get_logger(), "开始执行视觉伺服");
        
        // 图像中心点
        double center_x = image_width_ / 2.0;
        double center_y = image_height_ / 2.0;
        
        bool is_centered = false;
        
        // 只要YOLO还在处理视频就继续执行
        while (yolo_processing_) {
            // 计算目标点与图像中心的偏差
            double error_y = latest_target_center_.x - center_x;
            double error_x = latest_target_center_.y - center_y;

            // if (std::abs(error_x) > 200 || std::abs(error_y) > 200){
            if (std::abs(error_x) > 321 || std::abs(error_y) > 321){
                RCLCPP_ERROR(this->get_logger(), "当前偏差(像素) x=%.2f, y=%.2f",
                           error_x, error_y);
                RCLCPP_ERROR(this->get_logger(), "目标偏移过大，跳过当前round");
                continue;
            }

            // 检查是否已经居中
            if (std::abs(error_x) < position_tolerance_pixels_ && 
                std::abs(error_y) < position_tolerance_pixels_) {
                is_centered = true;
                RCLCPP_INFO(this->get_logger(), "目标已居中，偏差(像素): x=%.2f, y=%.2f", 
                           error_x, error_y);
                continue;
            }
            
            RCLCPP_INFO(this->get_logger(), "当前偏差(像素) x=%.2f, y=%.2f, 移动(米) x=%.4f, y=%.4f",
                       error_x, error_y, 
                       -error_x * pixel_to_meter_factor_, -error_y * pixel_to_meter_factor_);
            
            // 创建目标消息
            auto goal_msg = MoveEndToRelPos::Goal();
            // goal_msg.arm_id = "russ_group";
            goal_msg.arm_id = "ur_group";

            // 用于russ_group
            // goal_msg.pos.position.x = -error_x * pixel_to_meter_factor_;
            // goal_msg.pos.position.y = -error_y * pixel_to_meter_factor_;
            // goal_msg.pos.position.z = 0.0;

            // 用于ur_group图像跟踪测试
            goal_msg.pos.position.x = error_x * pixel_to_meter_factor_;
            goal_msg.pos.position.y = 0.0;
            goal_msg.pos.position.z = -error_y * pixel_to_meter_factor_;

            goal_msg.pos.orientation.x = 0.0;
            goal_msg.pos.orientation.y = 0.0;
            goal_msg.pos.orientation.z = 0.0;
            goal_msg.pos.orientation.w = 1.0;
            
            // 重置状态变量
            {
                std::lock_guard<std::mutex> lock(action_mutex_);
                action_completed_ = false;
                action_success_ = false;
                action_message_ = "";
            }
            
            // 设置回调并发送目标
            auto send_goal_options = rclcpp_action::Client<MoveEndToRelPos>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&EndControlService::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback =
                std::bind(&EndControlService::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&EndControlService::result_callback, this, std::placeholders::_1);
            
            auto goal_handle_future = rel_motion_client_->async_send_goal(goal_msg, send_goal_options);
            
            // 等待动作完成
            {
                std::unique_lock<std::mutex> lock(action_mutex_);
                if (!action_cv_.wait_for(lock, 
                                       std::chrono::seconds(30),
                                       [this] { return action_completed_; })) {
                    RCLCPP_ERROR(this->get_logger(), "等待Action结果超时");
                    return false;
                }
                
                if (!action_success_) {
                    RCLCPP_ERROR(this->get_logger(), "相对运动执行失败: %s", action_message_.c_str());
                    return false;
                }
            }
            
            // 等待新的目标检测结果
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        RCLCPP_INFO(this->get_logger(), "YOLO处理完成，视觉伺服结束");
        return is_centered;
    }

    bool perform_visual_servo_2()
    {
        RCLCPP_INFO(this->get_logger(), "开始执行视觉伺服");

        // 图像中心点
        double center_x = image_width_ / 2.0;
        double center_y = image_height_ / 2.0;

        bool is_centered = false;
        int attempts = 0;

        while (!is_centered && attempts < max_visual_servo_attempts_)
        {
            attempts++;

            // 计算目标点与图像中心的偏差
            double error_y = latest_target_center_.x - center_x;
            double error_x = latest_target_center_.y - center_y;

            RCLCPP_INFO(this->get_logger(), "视觉伺服尝试 %d: 偏差(像素) x=%.2f, y=%.2f, 移动(米) y=%.4f, z=%.4f",
                        attempts, error_x, error_y,
                        -error_x * pixel_to_meter_factor_, -error_y * pixel_to_meter_factor_);

            // 检查是否已经居中
            if (std::abs(error_x) < position_tolerance_pixels_ &&
                std::abs(error_y) < position_tolerance_pixels_)
            {
                is_centered = true;
                RCLCPP_INFO(this->get_logger(), "目标已居中，偏差(像素): x=%.2f, y=%.2f",
                            error_x, error_y);
                continue;
            }

            // 创建目标消息
            auto goal_msg = MoveEndToRelPos::Goal();
            goal_msg.arm_id = "russ_group";

            goal_msg.pos.position.x = -error_x * pixel_to_meter_factor_;
            goal_msg.pos.position.y = -error_y * pixel_to_meter_factor_;
            goal_msg.pos.position.z = 0.0;

            goal_msg.pos.orientation.x = 0.0;
            goal_msg.pos.orientation.y = 0.0;
            goal_msg.pos.orientation.z = 0.0;
            goal_msg.pos.orientation.w = 1.0;

            // 重置状态变量
            {
                std::lock_guard<std::mutex> lock(action_mutex_);
                action_completed_ = false;
                action_success_ = false;
                action_message_ = "";
            }

            // 设置回调并发送目标
            auto send_goal_options = rclcpp_action::Client<MoveEndToRelPos>::SendGoalOptions();
            send_goal_options.goal_response_callback =
                std::bind(&EndControlService::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback =
                std::bind(&EndControlService::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
                std::bind(&EndControlService::result_callback, this, std::placeholders::_1);

            auto goal_handle_future = rel_motion_client_->async_send_goal(goal_msg, send_goal_options);

            // 等待条件变量通知（由回调设置）
            {
                std::unique_lock<std::mutex> lock(action_mutex_);
                RCLCPP_INFO(this->get_logger(), "开始等待Action结果...");

                // 使用predicate避免虚假唤醒
                if (!action_cv_.wait_for(lock,
                                         std::chrono::seconds(30), // 30秒超时
                                         [this]
                                         { return action_completed_; }))
                {
                    RCLCPP_ERROR(this->get_logger(), "等待Action结果超时");
                    return false;
                }

                RCLCPP_INFO(this->get_logger(), "条件变量已被通知");
                RCLCPP_INFO(this->get_logger(), "收到Action结果: %s, 消息: %s",
                            action_success_ ? "成功" : "失败", action_message_.c_str());

                if (!action_success_)
                {
                    RCLCPP_ERROR(this->get_logger(), "相对运动执行失败: %s", action_message_.c_str());
                    return false;
                }
            }

            // 等待新的目标检测结果
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        if (is_centered)
        {
            RCLCPP_INFO(this->get_logger(), "视觉伺服成功：目标已居中");
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "视觉伺服未能完全对准目标，已达到最大尝试次数");
            return false;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 
        4  // 使用4个线程，可以根据需要调整
    );
    
    auto node = std::make_shared<EndControlService>();
    executor.add_node(node);
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
} 