/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-06 09:58:35
 * @LastEditors: “feiyang_hong” “feiyang.hong@infinityrobot.cn”
 * @LastEditTime: 2025-01-08 16:42:19
 * @FilePath: /planning_control_node/src/obstacle_node/src/obstacle_service.cpp
 * @Description: 
 *   障碍物管理服务节点，提供以下功能：
 *   1. 支持多种障碍物类型：
 *      - BOX: 长方体 [length, width, height]
 *      - CYLINDER: 圆柱体 [radius, height]
 *      - PLANE: 平面 [a, b, c, d] (ax + by + cz + d = 0)
 *   
 *   2. 提供三种基本操作：
 *      - ADD: 添加障碍物
 *      - REMOVE: 移除指定障碍物
 *      - CLEAR: 清除所有障碍物
 *   
 *   3. 预设障碍物：
 *      - ground_plane: 地面平面
 *      - box_1: 预设箱体
 *      - cylinder_1: 预设圆柱体
 *   
 *   使用方法：
 *   1. 启动节点：ros2 run obstacle_node obstacle_service
 *   2. 服务接口：manageObstacle (obstacle_node/srv/ManageObstacle)
 *   3. 服务调用示例：
 *      # 添加箱体
 *      ros2 service call /manageObstacle obstacle_node/srv/ManageObstacle "{
 *        operation: 1,
 *        obstacle_id: 'custom_box',
 *        primitive_type: 1,
 *        dimensions: [0.1, 0.1, 0.1],
 *        pose: {
 *          position: {x: 0.5, y: 0.0, z: 0.25},
 *          orientation: {w: 1.0}
 *        }
 *      }"
 *   
 *   注意事项：
 *   1. 所有障碍物都在 base_link 坐标系下
 *   2. 障碍物ID必须唯一
 *   3. 与MoveIt!集成，用于运动规划避障
 *   4. 支持动态添加和移除障碍物
 *   5. 节点启动时会自动添加预设障碍物
 */
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "obstacle_node/srv/manage_obstacle.hpp"
#include "obstacle_node/srv/manage_material.hpp"

class ObstacleService : public rclcpp::Node
{
public:
    ObstacleService() : Node("obstacle_service")
    {
        // 创建障碍物管理服务
        obstacle_service_ = this->create_service<obstacle_node::srv::ManageObstacle>(
            "manage_obstacle",
            std::bind(&ObstacleService::handle_obstacle_request, this,
                std::placeholders::_1, std::placeholders::_2));

        // 创建物料管理服务
        material_service_ = this->create_service<obstacle_node::srv::ManageMaterial>(
            "manage_material",
            std::bind(&ObstacleService::handle_material_request, this,
                std::placeholders::_1, std::placeholders::_2));

        planning_scene_interface_ = 
            std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // 添加预设障碍物
        add_predefined_obstacles();

        RCLCPP_INFO(this->get_logger(), "障碍物服务已启动");
    }

private:
    rclcpp::Service<obstacle_node::srv::ManageObstacle>::SharedPtr obstacle_service_;
    rclcpp::Service<obstacle_node::srv::ManageMaterial>::SharedPtr material_service_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::map<std::string, moveit_msgs::msg::CollisionObject> obstacle_map_;

    // 服务状态监控
    rclcpp::TimerBase::SharedPtr service_status_timer_;
    // bool service_running_ = true;
    std::atomic<bool> is_processing_{false};

    void check_service_status()
    {
        if (is_processing_) {
            RCLCPP_WARN(this->get_logger(), "物料管理服务正在处理请求中");
        }
    }

    void handle_obstacle_request(
        const std::shared_ptr<obstacle_node::srv::ManageObstacle::Request> request,
        std::shared_ptr<obstacle_node::srv::ManageObstacle::Response> response)
    {
        try {
            switch (request->operation) {
                case obstacle_node::srv::ManageObstacle::Request::ADD:
                    response->success = add_obstacle(
                        request->obstacle_id,
                        request->primitive_type,
                        request->dimensions,
                        request->pose
                    );
                    break;

                case obstacle_node::srv::ManageObstacle::Request::REMOVE:
                    response->success = remove_obstacle(request->obstacle_id);
                    break;

                case obstacle_node::srv::ManageObstacle::Request::CLEAR:
                    response->success = clear_obstacles();
                    break;

                default:
                    response->success = false;
                    response->message = "未知的操作类型";
                    return;
            }

            response->message = response->success ? "操作成功" : "操作失败";

        } catch (const std::exception& e) {
            response->success = false;
            response->message = "发生错误: " + std::string(e.what());
        }
    }

    void handle_material_request(
        const std::shared_ptr<obstacle_node::srv::ManageMaterial::Request> request,
        std::shared_ptr<obstacle_node::srv::ManageMaterial::Response> response)
    {
        is_processing_ = true;
        try {
            
            RCLCPP_INFO(this->get_logger(), "收到物料管理请求");
            // 添加处理时间日志
            auto start_time = this->now();

            switch (request->operation) {
                case obstacle_node::srv::ManageMaterial::Request::ADD_MATERIAL:
                    response->success = add_material(
                        request->material_id,
                        request->dimensions,
                        request->pose,
                        request->safety_margin);
                    RCLCPP_INFO(this->get_logger(), "收到添加物料请求");
                    break;

                case obstacle_node::srv::ManageMaterial::Request::REMOVE_MATERIAL:
                    response->success = remove_material(request->material_id);
                    RCLCPP_INFO(this->get_logger(), "收到移除物料请求");
                    break;

                case obstacle_node::srv::ManageMaterial::Request::UPDATE_POSE:
                    response->success = update_material_pose(
                        request->material_id,
                        request->pose);
                    RCLCPP_INFO(this->get_logger(), "收到更新物料位姿请求");
                    break;

                default:
                    response->success = false;
                    response->message = "未知的操作类型";
                    RCLCPP_INFO(this->get_logger(), "收到未知操作请求");
                    return;
            }

            if (response->success) {
                response->message = "操作成功";
                RCLCPP_INFO(this->get_logger(), "物料管理请求处理成功");
            }
            auto end_time = this->now();
            auto duration = (end_time - start_time).seconds();
            RCLCPP_INFO(this->get_logger(), "物料管理请求处理完成，耗时: %.3f秒", duration);
        

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("操作失败: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "处理物料请求异常: %s", e.what());
        }
        is_processing_ = false;
    }

    bool add_obstacle(
        const std::string& obstacle_id,
        uint8_t primitive_type,
        const std::vector<double>& dimensions,
        const geometry_msgs::msg::Pose& pose)
    {
        try {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "world";
            collision_object.id = obstacle_id;

            shape_msgs::msg::SolidPrimitive primitive;
            switch (primitive_type) {
                case obstacle_node::srv::ManageObstacle::Request::BOX:
                    primitive.type = primitive.BOX;
                    // primitive.dimensions = dimensions;  // [length, width, height]
                    // 清除并重新设置尺寸
                    primitive.dimensions.clear();
                    for(const auto& dim : dimensions) {
                        primitive.dimensions.push_back(dim);
                    }
                    break;

                case obstacle_node::srv::ManageObstacle::Request::CYLINDER:
                    primitive.type = primitive.CYLINDER;
                    // primitive.dimensions = dimensions;  // [radius, height]
                    // 清除并重新设置尺寸
                    primitive.dimensions.clear();
                    for(const auto& dim : dimensions) {
                        primitive.dimensions.push_back(dim);
                    }
                    break;

                case obstacle_node::srv::ManageObstacle::Request::PLANE:
                    // 处理平面类型
                    shape_msgs::msg::Plane plane;
                    // 直接赋值平面系数
                    if (dimensions.size() >= 4) {
                        plane.coef[0] = dimensions[0];  // a
                        plane.coef[1] = dimensions[1];  // b
                        plane.coef[2] = dimensions[2];  // c
                        plane.coef[3] = dimensions[3];  // d
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "平面参数数量不足");
                        return false;
                    }
                    collision_object.planes.push_back(plane);
                    collision_object.plane_poses.push_back(pose);
                    collision_object.operation = collision_object.ADD;
                    
                    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
                    collision_objects.push_back(collision_object);
                    planning_scene_interface_->addCollisionObjects(collision_objects);
                    
                    obstacle_map_[obstacle_id] = collision_object;
                    break;
            }

            if (primitive_type != obstacle_node::srv::ManageObstacle::Request::PLANE) {
                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(pose);
                collision_object.operation = collision_object.ADD;

                std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
                collision_objects.push_back(collision_object);
                planning_scene_interface_->addCollisionObjects(collision_objects);

                obstacle_map_[obstacle_id] = collision_object;
            }

            RCLCPP_INFO(this->get_logger(), "已添加障碍物: %s", obstacle_id.c_str());

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "添加障碍物失败: %s", e.what());
            return false;
        }
    }

    // 添加验证函数
    bool verify_obstacle_exists(const std::string &obstacle_id)
    {
        try
        {
            // 等待一小段时间让更改生效
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 获取当前场景中的所有碰撞对象
            auto collision_objects = planning_scene_interface_->getObjects();

            // 检查特定障碍物是否存在
            bool exists = collision_objects.find(obstacle_id) != collision_objects.end();

            if (!exists)
            {
                RCLCPP_WARN(this->get_logger(), "障碍物 %s 不存在于场景中", obstacle_id.c_str());
            }

            return exists;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "验证障碍物存在时发生错误: %s", e.what());
            return false;
        }
    }

    bool remove_obstacle(const std::string& obstacle_id)
    {
        try {
            if (obstacle_map_.find(obstacle_id) == obstacle_map_.end()) {
                RCLCPP_WARN(this->get_logger(), "未找到障碍物: %s", obstacle_id.c_str());
                return false;
            }

            std::vector<std::string> obstacle_ids = {obstacle_id};
            planning_scene_interface_->removeCollisionObjects(obstacle_ids);
            obstacle_map_.erase(obstacle_id);

            RCLCPP_INFO(this->get_logger(), "已移除障碍物: %s", obstacle_id.c_str());
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "移除障碍物失败: %s", e.what());
            return false;
        }
    }

    bool clear_obstacles()
    {
        try {
            std::vector<std::string> obstacle_ids;
            for (const auto& obstacle : obstacle_map_) {
                obstacle_ids.push_back(obstacle.first);
            }

            if (!obstacle_ids.empty()) {
                planning_scene_interface_->removeCollisionObjects(obstacle_ids);
                obstacle_map_.clear();
            }

            RCLCPP_INFO(this->get_logger(), "已清除所有障碍物");

            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "清除障碍物失败: %s", e.what());
            return false;
        }
    }

    void add_predefined_obstacles()
    {
        // 添加地面
        std::vector<double> ground_plane_coef = {0.0, 0.0, 1.0, 0.0};
        geometry_msgs::msg::Pose ground_pose;
        ground_pose.position.x = 0.0;
        ground_pose.position.y = 0.0;
        ground_pose.position.z = 0.0;
        add_obstacle(
            "ground_plane",
            obstacle_node::srv::ManageObstacle::Request::PLANE,
            ground_plane_coef,
            ground_pose
        );

        // 尝试多次添加地面，直到成功
        int max_attempts = 3;
        bool success = false;

        for (int i = 0; i < max_attempts && !success; ++i)
        {
            success = add_obstacle(
                "ground_plane",
                obstacle_node::srv::ManageObstacle::Request::PLANE,
                ground_plane_coef,
                ground_pose);
        }
        verify_obstacle_exists("ground_plane");

        // 添加预设的箱体障碍物
        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = 1.5;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.25;
        box_pose.orientation.w = 1.0;

        add_obstacle(
            "box_1",
            obstacle_node::srv::ManageObstacle::Request::BOX,
            {0.1, 0.1, 0.1},  // 长宽高
            box_pose
        );

        verify_obstacle_exists("box_1");

        // 添加预设的圆柱体障碍物
        geometry_msgs::msg::Pose cylinder_pose;
        cylinder_pose.position.x = 1.5;
        cylinder_pose.position.y = 0.0;
        cylinder_pose.position.z = 0.35;
        cylinder_pose.orientation.w = 1.0;

        add_obstacle(
            "cylinder_1",
            obstacle_node::srv::ManageObstacle::Request::CYLINDER,
            {0.05, 0.05},  // 半径和高度
            cylinder_pose
        );

        verify_obstacle_exists("cylinder_1");
    }

    bool add_material(
        const std::string& material_id,
        const std::vector<double>& dimensions,
        const geometry_msgs::msg::Pose& pose,
        double safety_margin)
    {
        try {
            // 创建规划场景接口
            moveit_msgs::msg::PlanningScene planning_scene;
            planning_scene.is_diff = true;

            // 创建碰撞对象
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "world";
            collision_object.id = material_id;

            // 创建包络体积
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {
                dimensions[0] + 2 * safety_margin,
                dimensions[1] + 2 * safety_margin,
                dimensions[2] + 2 * safety_margin
            };

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
            collision_object.operation = collision_object.ADD;

            planning_scene.world.collision_objects.push_back(collision_object);

            // 设置允许碰撞矩阵
            moveit_msgs::msg::AllowedCollisionMatrix acm;
            
            // 添加所有需要允许碰撞的链接
            std::vector<std::string> links = {
                material_id,
                "ee_link",
                "wrist_3_link",
                "wrist_2_link",
                "wrist_1_link",
                "forearm_link",
                "screw_driver_link",
                "endeffector_camera_link",
                "camera_base_link",     // 添加相机基座
                // "camera_link",          // 添加相机链接
                // "tool0",               // 添加工具链接
                // "flange"              // 添加法兰盘链接
            };
            
            acm.entry_names = links;
            acm.entry_values.resize(links.size());
            
            // 设置碰撞矩阵
            for (size_t i = 0; i < links.size(); ++i) {
                acm.entry_values[i].enabled.resize(links.size(), false);
                for (size_t j = 0; j < links.size(); ++j) {
                    // 如果其中一个是物料，另一个是机器人链接，则允许碰撞
                    if ((links[i] == material_id && i != j) || 
                        (links[j] == material_id && i != j)) {
                        acm.entry_values[i].enabled[j] = true;
                    }
                }
            }

            planning_scene.allowed_collision_matrix = acm;

            // 发布规划场景更新
            moveit_msgs::msg::PlanningSceneWorld world;
            world.collision_objects.push_back(collision_object);
            planning_scene_interface_->addCollisionObjects(world.collision_objects);

            RCLCPP_INFO(this->get_logger(), "添加物料: %s，并设置允许碰撞", material_id.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "添加物料失败: %s", e.what());
            return false;
        }
    }

    bool remove_material(const std::string& material_id)
    {
        try {
            std::vector<std::string> object_ids;
            object_ids.push_back(material_id);
            planning_scene_interface_->removeCollisionObjects(object_ids);

            RCLCPP_INFO(this->get_logger(), "移除物料: %s", material_id.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "移除物料失败: %s", e.what());
            return false;
        }
    }

    bool update_material_pose(
        const std::string& material_id,
        const geometry_msgs::msg::Pose& pose)
    {
        try {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id = "world";
            collision_object.id = material_id;
            collision_object.operation = collision_object.MOVE;
            collision_object.primitive_poses.push_back(pose);

            std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
            collision_objects.push_back(collision_object);
            planning_scene_interface_->addCollisionObjects(collision_objects);

            RCLCPP_INFO(this->get_logger(), "更新物料位姿: %s", material_id.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "更新物料位姿失败: %s", e.what());
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleService>();
    // rclcpp::spin(node);

    // 使用多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
} 