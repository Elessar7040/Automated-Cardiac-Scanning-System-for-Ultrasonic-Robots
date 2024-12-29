#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class MoveItPlanner {
private:
    rclcpp::Node::SharedPtr node_;
    // 将 logger_ 类型改为 const 引用
    const rclcpp::Logger& logger_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

public:
    MoveItPlanner(const std::string& node_name, const std::string& group_name) 
        : logger_(node_->get_logger())
    {
        // 初始化节点
        node_ = std::make_shared<rclcpp::Node>(
            node_name,
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        
        // 初始化 MoveGroup
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            node_, 
            group_name
        );
    }

    geometry_msgs::msg::Pose createTargetPose(
        double x, double y, double z, double w = 1.0) {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = w;
        msg.position.x = x;
        msg.position.y = y;
        msg.position.z = z;
        return msg;
    }

    bool planAndExecute(const geometry_msgs::msg::Pose& target_pose) {
        move_group_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_->plan(plan));

        if (success) {
            move_group_->execute(plan);
            return true;
        } else {
            RCLCPP_ERROR(logger_, "Planning failed!");
            return false;
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // 创建规划器实例
    MoveItPlanner planner("moveit_cartesian_test_2", "ur_group");

    // 创建目标姿态
    auto target_pose = planner.createTargetPose(0.4, 0.1, 0.5);
    
    // 执行规划和运动
    planner.planAndExecute(target_pose);
    
    rclcpp::shutdown();
    return 0;
}