/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 12:19:02
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-03 13:38:03
 * @FilePath: /planning_control_node/src/planning_node/include/planning_node/action_server_base.hpp
 * @Description: Action服务端的基类，提供共享的运动参数和避障设置
 * 
 * Service 服务:
 *   - glob_planning_setting: 用于配置全局避障功能
 *     - 输入: obstacle_avoidance_enabled (避障开关), planning_mode (规划模式)
 *     - 输出: success (配置结果), message (结果信息)
 *     - 规划模式:
 *       - 0: 普通模式 (默认，中等速度和精度)
 *       - 1: 安全模式 (低速，高精度)
 *       - 2: 快速模式 (高速，低精度)
 *       - 3: 精确模式 (低速，超高精度)
 *
 * 避障功能:
 *   - 开启时: 使用完整的避障规划，考虑所有场景中的障碍物
 *   - 关闭时: 临时移除所有障碍物，直接规划运动路径
 *   - 重新开启时: 恢复之前保存的场景障碍物
 */

#ifndef ACTION_SERVER_BASE_HPP_
#define ACTION_SERVER_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "planning_node/srv/glob_planning_setting.hpp"

class ActionServerBase : public rclcpp::Node
{
protected:
    ActionServerBase(const std::string& node_name) : Node(node_name)
    {
        // 创建全局规划设置服务
        planning_setting_srv_ = this->create_service<planning_node::srv::GlobPlanningSetting>(
            "glob_planning_setting",
            std::bind(&ActionServerBase::handle_planning_setting, this, 
                std::placeholders::_1, std::placeholders::_2));
    }

    // 配置MoveGroup的共享参数
    void configure_move_group(
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group)
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        
        if (obstacle_avoidance_enabled_) {
            move_group->setPlannerId("RRTConnect");
        } else {
            move_group->setPlannerId("RRTConnect");
            move_group->clearPathConstraints();
        }

        // 根据规划模式设置参数
        switch (planning_mode_) {
            case 0:  // 普通模式
                move_group->setMaxVelocityScalingFactor(0.3);
                move_group->setMaxAccelerationScalingFactor(0.3);
                break;
            case 1:  // 安全模式
                move_group->setMaxVelocityScalingFactor(0.1);
                move_group->setMaxAccelerationScalingFactor(0.1);
                break;
            case 2:  // 快速模式
                move_group->setMaxVelocityScalingFactor(0.8);
                move_group->setMaxAccelerationScalingFactor(0.8);
                break;
            case 3:  // 精确模式
                move_group->setMaxVelocityScalingFactor(0.2);
                move_group->setMaxAccelerationScalingFactor(0.2);
                move_group->setGoalPositionTolerance(0.001);
                break;
        }
    }

    // 处理避障设置的服务回调
    void handle_planning_setting(
        const std::shared_ptr<planning_node::srv::GlobPlanningSetting::Request> request,
        std::shared_ptr<planning_node::srv::GlobPlanningSetting::Response> response)
    {
        std::lock_guard<std::mutex> lock(planning_mutex_);
        
        try {
            // 先获取当前值用于日志
            bool new_enabled = request->obstacle_avoidance_enabled;
            uint8_t new_mode = request->planning_mode;
            
            // 如果状态发生改变
            if (new_enabled != obstacle_avoidance_enabled_) {
                RCLCPP_INFO(this->get_logger(), "避障状态发生改变: %s -> %s",
                obstacle_avoidance_enabled_ ? "开启" : "关闭",
                new_enabled ? "开启" : "关闭");
                
                auto planning_scene_interface = 
                    std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
                
                // 等待场景接口初始化
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                if (!new_enabled) {
                    // 关闭避障时，保存当前场景
                    if (!has_saved_scene_) {
                        // 获取当前所有碰撞对象
                        // std::vector<std::string> object_ids = 
                        //     planning_scene_interface->getKnownObjectNames();
                        std::vector<std::string> object_ids;
                        try {
                            object_ids = planning_scene_interface->getKnownObjectNames();
                            RCLCPP_INFO(this->get_logger(), "获取到 %ld 个场景对象", object_ids.size());
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(this->get_logger(), "获取场景对象失败: %s", e.what());
                            object_ids.clear();
                        }
                        if (!object_ids.empty()) {
                            saved_collision_objects_.clear();  // 清空之前可能的残留
                            try {
                                // 一次性获取所有对象
                                auto objects_map = planning_scene_interface->getObjects(object_ids);
                                // 将map中的对象转换为vector
                                for (const auto& [id, obj] : objects_map) {
                                    saved_collision_objects_.push_back(obj);
                                    RCLCPP_INFO(this->get_logger(), "已保存对象: %s", id.c_str());
                                }
                                has_saved_scene_ = true;
                                RCLCPP_INFO(this->get_logger(), 
                                    "成功保存 %ld 个场景对象", 
                                    saved_collision_objects_.size());
                            
                                // if (!objects.empty()) {
                                //     saved_collision_objects_ = objects;
                                //     has_saved_scene_ = true;
                                //     RCLCPP_INFO(this->get_logger(), "成功保存 %ld 个场景对象", objects.size());
                                // }
                            } catch (const std::exception& e) {
                                RCLCPP_ERROR(this->get_logger(), "保存场景对象失败: %s", e.what());
                            }
                            // for (const auto& id : object_ids) {
                            //     try {
                            //         auto objects = planning_scene_interface->getObjects({id});
                            //         if (!objects.empty()) {
                            //             saved_collision_objects_.push_back(objects[0]);
                            //             RCLCPP_INFO(this->get_logger(), "已保存对象: %s", id.c_str());
                            //         }
                                    
                            //     } catch (const std::exception& e) {
                            //         RCLCPP_WARN(this->get_logger(), "保存对象 %s 失败: %s", 
                            //             id.c_str(), e.what());
                            //     }
                            // }
                            // has_saved_scene_ = true;
                        
                        // 保存当前场景信息
                        // for (const auto& id : object_ids) {
                        //     auto collision_object = 
                        //         planning_scene_interface->getObjects({id})[0];
                        //     saved_collision_objects_.push_back(collision_object);
                        // }
                        // has_saved_scene_ = true;
                        }
                    }
                    // 清除当前场景
                    try {
                        std::vector<std::string> current_objects = 
                            planning_scene_interface->getKnownObjectNames();
                        if (!current_objects.empty()) {
                            planning_scene_interface->removeCollisionObjects(current_objects);
                            RCLCPP_INFO(this->get_logger(), "已清除 %ld 个场景对象", 
                                current_objects.size());
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "清除场景对象失败: %s", e.what());
                    }
                } else {
                    // 开启避障时，恢复保存的场景
                    // if (has_saved_scene_) {
                    //     // 先清除当前场景
                    //     std::vector<std::string> current_objects = 
                    //         planning_scene_interface->getKnownObjectNames();
                    //     planning_scene_interface->removeCollisionObjects(current_objects);
                    //     // 恢复保存的障碍物
                    //     planning_scene_interface->addCollisionObjects(saved_collision_objects_);
                        
                    //     has_saved_scene_ = false;
                    //     saved_collision_objects_.clear();
                    // }
                    if (has_saved_scene_ && !saved_collision_objects_.empty()) {
                        try {
                            planning_scene_interface->addCollisionObjects(saved_collision_objects_);
                            RCLCPP_INFO(this->get_logger(), "已恢复 %ld 个障碍物", 
                                saved_collision_objects_.size());
                            has_saved_scene_ = false;
                            saved_collision_objects_.clear();
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(this->get_logger(), "恢复场景对象失败: %s", e.what());
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "没有找到保存的场景信息");
                    }
                }
            }
            
            // 更新配置
            obstacle_avoidance_enabled_ = new_enabled;
            planning_mode_ = new_mode;
            
            response->success = true;
            response->message = "配置已更新";
            
            RCLCPP_INFO(this->get_logger(), 
                "更新全局避障配置：避障=%s, 模式=%d", 
                new_enabled ? "开启" : "关闭", 
                new_mode);
        }
        catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("配置更新失败: ") + e.what();
        }
    }

protected:
    std::mutex planning_mutex_;
    bool obstacle_avoidance_enabled_{true};
    uint8_t planning_mode_{0};
    std::vector<moveit_msgs::msg::CollisionObject> saved_collision_objects_;
    bool has_saved_scene_{false};

private:
    rclcpp::Service<planning_node::srv::GlobPlanningSetting>::SharedPtr planning_setting_srv_;
};

#endif  // ACTION_SERVER_BASE_HPP_ 