#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher() : Node("pose_publisher"), current_target_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),  // 每5秒发送一次新的目标位姿
            std::bind(&PosePublisher::timer_callback, this));
        
        // 预设目标点
        target_poses_ = {
            {0.3, 0.3, 0.3},
            {0.4, 0.4, 0.3},
            {0.5, 0.4, 0.3},
            {0.5, 0.5, 0.3},
            {0.3, 0.5, 0.3}
        };
            
        RCLCPP_INFO(this->get_logger(), "位姿发布节点已启动");
    }

private:
    const double position_offset_ = 0.05;  // 添加偏移量作为成员变量
    int count = 0;

    std::vector<std::array<double, 3>> target_poses_;  // 存储所有目标点
    size_t current_target_;  // 当前目标点索引

    void timer_callback()
    {
        auto message = geometry_msgs::msg::PoseStamped();
        message.header.stamp = this->now();
        message.header.frame_id = "world";  // 设置坐标系

        // 设置目标位姿（这里可以根据需要修改）
        
        
        // message.pose.position.x = 0.3 + count * position_offset_;
        // message.pose.position.y = 0.3 + count * position_offset_;
        // message.pose.position.z = 0.3 + count * position_offset_;

        message.pose.position.x = target_poses_[current_target_][0];
        message.pose.position.y = target_poses_[current_target_][1];
        message.pose.position.z = target_poses_[current_target_][2];
        
        message.pose.orientation.w = 1.0;
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "发布新的目标位姿 %ld:: x=%.2f, y=%.2f, z=%.2f",
            current_target_ + 1,
            message.pose.position.x, 
            message.pose.position.y, 
            message.pose.position.z);

        current_target_ = (current_target_ + 1) % target_poses_.size();

        if (message.pose.position.x >= 0.6 || message.pose.position.y >= 0.6 || message.pose.position.z >= 0.6)
        {
            count = 0;
        }
        else
        {
            count++;
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 