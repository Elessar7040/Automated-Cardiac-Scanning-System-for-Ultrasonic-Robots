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

// 使用end_control_node自己的action定义
#include "end_control_node/action/move_end_to_rel_pos.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

class EndControlService : public rclcpp::Node
{
public:
    using MoveEndToRelPos = end_control_node::action::MoveEndToRelPos;
    using GoalHandleMoveEndToRelPos = rclcpp_action::ClientGoalHandle<MoveEndToRelPos>;
    
    EndControlService() : Node("end_control_service")
    {
        // 创建服务
        service_ = this->create_service<end_control_node::srv::EndControl>(
            "end_control",
            std::bind(&EndControlService::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));

        // 订阅YOLO目标中心点话题
        target_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/yolo/target_center", 10,
            std::bind(&EndControlService::target_callback, this, std::placeholders::_1));
            
        // 订阅YOLO处理后的图像
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/yolo/detected_image", 10,
            std::bind(&EndControlService::image_callback, this, std::placeholders::_1));
            
        // 创建相对运动action客户端
        rel_motion_client_ = rclcpp_action::create_client<MoveEndToRelPos>(
            this, "move_end_to_rel_pos");

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
    int image_width_ = 640;  // 默认值
    int image_height_ = 480; // 默认值
    bool has_image_ = false;
    
    // 视觉伺服参数
    const double pixel_to_meter_factor_ = 0.0005;  // 像素到米的转换因子
    const int max_visual_servo_attempts_ = 5;      // 最大视觉伺服尝试次数
    const double position_tolerance_pixels_ = 10.0; // 位置容差（像素）
    
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
        // 使用 initial_pose 参数
        RCLCPP_INFO(this->get_logger(), "执行探针控制，目标位置: [%f, %f, %f]",
            initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
        
        // 检查是否有目标点
        if (!has_target_) {
            RCLCPP_ERROR(this->get_logger(), "未检测到目标点，无法执行视觉伺服");
            return false;
        }
        
        // 执行视觉伺服
        bool servo_success = perform_visual_servo();
        if (!servo_success) {
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
        
        if (!has_target_ || !has_image_) {
            RCLCPP_ERROR(this->get_logger(), "缺少目标点或图像信息，无法执行视觉伺服");
            return false;
        }
        
        // 图像中心点
        double center_x = image_width_ / 2.0;
        double center_y = image_height_ / 2.0;
        
        bool is_centered = false;
        int attempts = 0;
        
        while (!is_centered && attempts < max_visual_servo_attempts_) {
            attempts++;
            
            // 计算目标点与图像中心的偏差
            double error_x = latest_target_center_.x - center_x;
            double error_y = latest_target_center_.y - center_y;
            
            // 检查是否已经居中
            if (std::abs(error_x) < position_tolerance_pixels_ && 
                std::abs(error_y) < position_tolerance_pixels_) {
                is_centered = true;
                RCLCPP_INFO(this->get_logger(), "目标已居中，偏差(像素): x=%.2f, y=%.2f", 
                           error_x, error_y);
                continue;
            }
            
            // 计算相对运动（像素到米的转换）
            // 注意：相机坐标系与机械臂坐标系的对应关系
            // 假设：相机x轴对应机械臂-y轴，相机y轴对应机械臂-z轴
            double move_y = -error_x * pixel_to_meter_factor_;
            double move_z = -error_y * pixel_to_meter_factor_;
            
            RCLCPP_INFO(this->get_logger(), 
                       "视觉伺服尝试 %d: 偏差(像素) x=%.2f, y=%.2f, 移动(米) y=%.4f, z=%.4f", 
                       attempts, error_x, error_y, move_y, move_z);
            
            // 模拟移动，不实际调用服务
            // 在实际应用中，这里应该调用moveEndToRelPos服务
            
            // 模拟等待新的目标检测结果
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 模拟移动后的新位置（假设移动成功）
            latest_target_center_.x = latest_target_center_.x - error_x * 0.5;
            latest_target_center_.y = latest_target_center_.y - error_y * 0.5;
        }
        
        if (is_centered) {
            RCLCPP_INFO(this->get_logger(), "视觉伺服成功：目标已居中");
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "视觉伺服未能完全对准目标，已达到最大尝试次数");
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EndControlService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 