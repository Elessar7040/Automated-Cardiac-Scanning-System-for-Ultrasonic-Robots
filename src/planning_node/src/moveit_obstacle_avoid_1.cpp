#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>

class MoveitObstacleAvoid : public rclcpp::Node
{
public:
    MoveitObstacleAvoid() : Node("moveit_obstacle_avoid_1")
    {
        // 延迟初始化 move_group_ptr_
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 500ms后初始化
            std::bind(&MoveitObstacleAvoid::init_move_group, this));
        
        // 添加场景接口
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        RCLCPP_INFO(this->get_logger(), "节点已启动，等待初始化...");
    }

private:
    void init_move_group()
    {
        // 只执行一次
        timer_->cancel();

        // 在这里初始化 move_group_ptr_
        move_group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ur_group");

        // 设置规划参数
        move_group_ptr_->setPlannerId("RRTConnect");  // 使用RRT算法
        move_group_ptr_->setNumPlanningAttempts(10);  // 尝试次数
        move_group_ptr_->setPlanningTime(10.0);        // 规划时间限制
        move_group_ptr_->setMaxVelocityScalingFactor(0.3);
        move_group_ptr_->setMaxAccelerationScalingFactor(0.3);

        // 允许重规划
        move_group_ptr_->allowReplanning(true);
        
        // 创建订阅者
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&MoveitObstacleAvoid::pose_callback, this, std::placeholders::_1)
        );

        // 设置更多规划参数
        move_group_ptr_->setGoalTolerance(0.01);  // 设置目标容差
        
        // 添加示例障碍物
        add_obstacles();
        
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
        box.id = "box1";

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
        } else {
            RCLCPP_ERROR(this->get_logger(), "避障规划失败，错误码: %d", error_code.val);
        }
    }

    // 添加 timer_ 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveitObstacleAvoid>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}