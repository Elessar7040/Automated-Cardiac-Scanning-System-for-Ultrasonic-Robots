/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2024-12-27 16:54:58
 * @LastEditors: “feiyang_hong” “feiyang.hong@infinityrobot.cn”
 * @LastEditTime: 2025-01-08 17:28:32
 * @FilePath: /planning_control_node/src/planning_node/src/moveit_obstacle_avoid_2.cpp
 * @Description: 通过订阅"target_pose"话题，MoveitObstacleAvoid节点订阅目标位姿，添加障碍物，并进行避障规划和运动执行
 * 由Moveit_node/src/moveit_env.launch.py启动
 * 加入pybind11模块，链接python接口中FRRobot_test类
 */


#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>


class MoveitObstacleAvoid : public rclcpp::Node
{
public:
    MoveitObstacleAvoid() : Node("moveit_obstacle_avoid_2")
    {
        // 延迟初始化 move_group_ptr_
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 500ms后初始化
            std::bind(&MoveitObstacleAvoid::init_move_group, this));
        
        // 添加场景接口
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        RCLCPP_INFO(this->get_logger(), "节点已启动，等待初始化...");
    }
    ~MoveitObstacleAvoid() = default;

private:
    void init_move_group()
    {
        // 只执行一次
        timer_->cancel();
        try {
            // 从Python对象获取机器人组名称
            std::string group_name = "ur_group";
            
            // 使用获取到的组名初始化 move_group_ptr_
            move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), group_name);

            // 设置规划参数
            move_group_ptr_->setPlannerId("RRTConnect");  // 使用RRT算法
            move_group_ptr_->setNumPlanningAttempts(10);  // 尝试次数
            move_group_ptr_->setPlanningTime(10.0);        // 规划时间限制
            move_group_ptr_->setMaxVelocityScalingFactor(0.3);
            move_group_ptr_->setMaxAccelerationScalingFactor(0.3);
            // 允许重规划
            move_group_ptr_->allowReplanning(true);

            RCLCPP_INFO(this->get_logger(), "使用组名 '%s' 初始化成功", group_name.c_str());
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "初始化失败: %s", e.what());
        }

        // 在这里初始化 move_group_ptr_
        // move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        //     shared_from_this(), "fairino5_v6_group");

        // 设置规划参数
        // move_group_ptr_->setPlannerId("RRTConnect");  // 使用RRT算法
        // move_group_ptr_->setNumPlanningAttempts(10);  // 尝试次数
        // move_group_ptr_->setPlanningTime(10.0);        // 规划时间限制
        // move_group_ptr_->setMaxVelocityScalingFactor(0.8);
        // move_group_ptr_->setMaxAccelerationScalingFactor(0.8);

        // 允许重规划
        // move_group_ptr_->allowReplanning(true);
        
        // 创建订阅者
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&MoveitObstacleAvoid::pose_callback, this, std::placeholders::_1)
        );

        // 设置更多规划参数
        move_group_ptr_->setGoalTolerance(0.01);  // 设置目标容差

        // 添加地面约束
        add_ground_plane();
        
        // 添加示例障碍物
        add_obstacles();

        // 添加一个圆柱形障碍物
        // std::string obstacle_id = "box_1";
        std::string obstacle_id = "cylinder_1";

        geometry_msgs::msg::Point cylinder_position;
        cylinder_position.x = 0.5;
        cylinder_position.y = 0.0;
        cylinder_position.z = 0.35;

        geometry_msgs::msg::Quaternion cylinder_orientation;
        cylinder_orientation.x = 0.0;
        cylinder_orientation.y = 0.0;
        cylinder_orientation.z = 0.0;
        cylinder_orientation.w = 1.0;

        add_obstacle(
            obstacle_id, 
            // shape_msgs::msg::SolidPrimitive::BOX,
            // {0.1, 0.1, 0.1}, 
            shape_msgs::msg::SolidPrimitive::CYLINDER,
            {0.05, 0.05}, 
            cylinder_position, 
            cylinder_orientation
        );
        
        
        RCLCPP_INFO(this->get_logger(), "节点初始化完成，场景中已添加障碍物");
    }

    void add_obstacles()
    {
        // 等待场景接口准备就绪
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 创建一个障碍物列表
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        
        // 添加一个箱形障碍物
        moveit_msgs::msg::CollisionObject box;
        box.header.frame_id = move_group_ptr_->getPlanningFrame();
        box.id = "box_1";

        // 定义箱体的大小和位置
        shape_msgs::msg::SolidPrimitive box_primitive;
        box_primitive.type = box_primitive.BOX;
        box_primitive.dimensions = {0.1, 0.1, 0.1};  // 长宽高

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = 0.5;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.25;
        box_pose.orientation.w = 1.0;

        box.primitives.push_back(box_primitive);
        box.primitive_poses.push_back(box_pose);
        box.operation = box.ADD;

        collision_objects.push_back(box);

        // 添加到场景中
        planning_scene_interface_->addCollisionObjects(collision_objects);

        RCLCPP_INFO(this->get_logger(), "已添加障碍物: box_1");
    }

    /**
     * @description: 添加障碍物
     * @param {string&} obstacle_id: 障碍物的ID
     * @param {int} primitive_type: 障碍物类型
     * @param {vector<double>&} dimensions: 障碍物尺寸
     * @param {Point&} position: 障碍物位置
     * @param {Quaternion&} orientation: 障碍物方向，使用四元数
     * @return {*}
     * 
     * @example
     * // 添加一个箱体障碍物
     * std::string box_id = "box_1";
     * geometry_msgs::msg::Point box_position;
     * box_position.x = 0.5;
     * box_position.y = 0.0;
     * box_position.z = 0.25;
     * 
     * geometry_msgs::msg::Quaternion box_orientation;
     * box_orientation.w = 1.0;
     * 
     * add_obstacle(
     *     box_id,
     *     shape_msgs::msg::SolidPrimitive::BOX,
     *     {0.1, 0.1, 0.1},  // 长宽高
     *     box_position,
     *     box_orientation
     * );
     * 
     * // 添加一个圆柱体障碍物
     * std::string cylinder_id = "cylinder_1";
     * geometry_msgs::msg::Point cylinder_position;
     * cylinder_position.x = 0.5;
     * cylinder_position.y = 0.0;
     * cylinder_position.z = 0.35;
     * 
     * geometry_msgs::msg::Quaternion cylinder_orientation;
     * cylinder_orientation.w = 1.0;
     * 
     * add_obstacle(
     *     cylinder_id,
     *     shape_msgs::msg::SolidPrimitive::CYLINDER,
     *     {0.05, 0.05},  // 半径和高度
     *     cylinder_position,
     *     cylinder_orientation
     * );
     */
    void add_obstacle(
        const std::string& obstacle_id,
        int primitive_type,
        const std::vector<double>& dimensions,
        const geometry_msgs::msg::Point& position,
        const geometry_msgs::msg::Quaternion& orientation)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));

        // 创建一个障碍物列表
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        // 创建一个障碍物
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_ptr_->getPlanningFrame();
        collision_object.id = obstacle_id;

        // 创建一个障碍物的形状
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive_type;
        // primitive.dimensions = dimensions;
        // primitive.dimensions = {0.1, 0.1, 0.1};  // 箱体
        // primitive.dimensions = {0.1, 0.1};  // 圆柱体
        // 清除并重新设置尺寸
        primitive.dimensions.clear();
        for(const auto& dim : dimensions) {
            primitive.dimensions.push_back(dim);
        }

        // 创建一个障碍物的位置和方向
        geometry_msgs::msg::Pose pose;
        pose.position = position;
        pose.orientation = orientation;

        // 将形状和位置添加到障碍物中
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = collision_object.ADD;

        // 将障碍物添加到场景中
        collision_objects.push_back(collision_object);
        planning_scene_interface_->addCollisionObjects(collision_objects);

        RCLCPP_INFO(this->get_logger(), "已添加障碍物: %s", obstacle_id.c_str());

    }

    void add_ground_plane()
    {
        // 创建地面平面碰撞对象
        moveit_msgs::msg::CollisionObject ground_plane;
        ground_plane.header.frame_id = move_group_ptr_->getPlanningFrame();
        ground_plane.id = "ground_plane";

        // 定义平面的法向量和点（平面方程：ax + by + cz + d = 0）
        shape_msgs::msg::Plane plane;
        plane.coef = {0.0, 0.0, 1.0, 0.0};  // 参数为a, b, c, d，法向量：(0, 0, 1)，点：(0, 0, 0)

        // 设置碰撞对象的形状为平面
        ground_plane.planes.push_back(plane);
        ground_plane.plane_poses.push_back(geometry_msgs::msg::Pose());  // 默认位姿

        // 设置操作类型为添加
        ground_plane.operation = ground_plane.ADD;
        
        // 将地面平面添加到场景中
        std::vector<moveit_msgs::msg::CollisionObject> collision_ground_plane;
        collision_ground_plane.push_back(ground_plane);
        planning_scene_interface_->addCollisionObjects(collision_ground_plane);

        // 添加颜色
        moveit_msgs::msg::ObjectColor ground_color;
        ground_color.id = ground_plane.id;
        ground_color.color.r = 0.0;  // 红色分量
        ground_color.color.g = 0.8;  // 绿色分量
        ground_color.color.b = 0.0;  // 蓝色分量
        ground_color.color.a = 0.5;  // 透明度 (0-1)

        // 创建 PlanningScene 消息
        moveit_msgs::msg::PlanningScene planning_scene;
        planning_scene.is_diff = true;  // 这是一个差异更新
        planning_scene.object_colors.push_back(ground_color);

        // 发布颜色更新
        auto planning_scene_diff_publisher = 
            this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
        planning_scene_diff_publisher->publish(planning_scene);

        RCLCPP_INFO(this->get_logger(), "已添加地面约束");

        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void add_complex_constraints()
    {
        // 添加地面
        moveit_msgs::msg::CollisionObject ground;
        ground.header.frame_id = move_group_ptr_->getPlanningFrame();
        ground.id = "ground";
        shape_msgs::msg::Plane ground_plane;
        ground_plane.coef = {0.0, 0.0, 1.0, 0.0};
        ground.planes.push_back(ground_plane);
        ground.plane_poses.push_back(geometry_msgs::msg::Pose());
        ground.operation = ground.ADD;
    
        // 添加侧面墙
        moveit_msgs::msg::CollisionObject wall;
        wall.header.frame_id = move_group_ptr_->getPlanningFrame();
        wall.id = "wall";
        shape_msgs::msg::Plane wall_plane;
        wall_plane.coef = {1.0, 0.0, 0.0, -1.0};  // x = 1 平面
        wall.planes.push_back(wall_plane);
        wall.plane_poses.push_back(geometry_msgs::msg::Pose());
        wall.operation = wall.ADD;
    
        // 将所有约束添加到场景中
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(ground);
        collision_objects.push_back(wall);
        planning_scene_interface_->addCollisionObjects(collision_objects);
    
        RCLCPP_INFO(this->get_logger(), "已添加复杂约束");
    }

    void set_object_color(
        const std::string& object_id, 
        double r, double g, double b, 
        double alpha = 0.5)
    {
        // 创建颜色消息
        moveit_msgs::msg::ObjectColor object_color;
        object_color.id = object_id;
        object_color.color.r = r;
        object_color.color.g = g;
        object_color.color.b = b;
        object_color.color.a = alpha;

        // 创建 PlanningScene 消息
        moveit_msgs::msg::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.object_colors.push_back(object_color);

        // 发布颜色更新
        auto planning_scene_diff_publisher = 
            this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

        rclcpp::sleep_for(std::chrono::seconds(1));
        planning_scene_diff_publisher->publish(planning_scene);
    }
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到新的目标位姿，开始避障规划...");

        // 设置目标位姿
        move_group_ptr_->setPoseTarget(msg->pose);

        // 进行运动规划
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        // 尝试规划避障路径
        auto error_code = move_group_ptr_->plan(my_plan);
        bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "避障规划成功，开始执行运动...");
            move_group_ptr_->execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "运动执行完成！");
            rclcpp::sleep_for(std::chrono::seconds(2));
        } else {
            RCLCPP_ERROR(this->get_logger(), "避障规划失败，错误码: %d", error_code.val);
        }
    }

    // 添加pybind11模块定义
    // PYBIND11_MODULE(moveit_obstacle_avoid_2, m) {
    //     py::class_<MoveitObstacleAvoid>(m, "MoveitObstacleAvoid")
    //         .def(py::init<>())
    //         .def("add_obstacle", &MoveitObstacleAvoid::add_obstacle);
    // }

    // 添加 timer_ 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitObstacleAvoid>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}