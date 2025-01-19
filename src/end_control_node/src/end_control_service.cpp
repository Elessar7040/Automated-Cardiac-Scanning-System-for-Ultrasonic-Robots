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

class EndControlService : public rclcpp::Node
{
public:
    EndControlService() : Node("end_control_service")
    {
        // 创建服务
        service_ = this->create_service<end_control_node::srv::EndControl>(
            "end_control",
            std::bind(&EndControlService::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "末端控制服务已启动");
    }

private:
    rclcpp::Service<end_control_node::srv::EndControl>::SharedPtr service_;
    
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

                case end_control_node::srv::EndControl::Request::TYPE_GRIPPER:
                    response->success = control_gripper(
                        request->device_id,
                        request->action);
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

    bool control_gripper(uint8_t gripper_id, double position)
    {
        RCLCPP_INFO(this->get_logger(), 
            "控制夹爪 %d: 位置 %.2f", 
            gripper_id, 
            position);

        // TODO: 在这里添加实际的硬件控制代码
        // 例如：通过 CAN 或其他接口控制夹爪


        
        return true;  // 暂时返回成功
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