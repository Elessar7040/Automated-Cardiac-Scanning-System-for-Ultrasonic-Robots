#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <deque>

class MatchPointConverter : public rclcpp::Node
{
public:
    MatchPointConverter() : Node("match_point_converter")
    {
        // 声明并获取参数
        this->declare_parameter("filter_window_size", 5);  // 滤波窗口大小
        this->declare_parameter("position_threshold", 0.02);  // 位置变化阈值（米）
        
        filter_window_size_ = this->get_parameter("filter_window_size").as_int();
        position_threshold_ = this->get_parameter("position_threshold").as_double();

        // 创建订阅者
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "match_point", 10, 
            std::bind(&MatchPointConverter::match_point_callback, this, std::placeholders::_1));

        // 创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10);
        
        RCLCPP_INFO(this->get_logger(), "Match Point Converter节点已启动");
        RCLCPP_INFO(this->get_logger(), "滤波窗口大小: %d", filter_window_size_);
        RCLCPP_INFO(this->get_logger(), "位置变化阈值: %.3f", position_threshold_);
    }

private:
    void match_point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // 添加新的点到缓存
        point_buffer_.push_back(*msg);
        
        // 保持缓存大小
        while (point_buffer_.size() > filter_window_size_) {
            point_buffer_.pop_front();
        }

        // 如果缓存中的点不够，直接返回
        if (point_buffer_.size() < filter_window_size_) {
            return;
        }

        // 计算均值
        geometry_msgs::msg::Point filtered_point;
        filtered_point.x = 0.0;
        filtered_point.y = 0.0;
        filtered_point.z = 0.0;

        for (const auto& point : point_buffer_) {
            filtered_point.x += point.x;
            filtered_point.y += point.y;
            filtered_point.z += point.z;
        }

        filtered_point.x /= point_buffer_.size();
        filtered_point.y /= point_buffer_.size();
        filtered_point.z /= point_buffer_.size();

        filtered_point.y = 0.2;

        // 检查位置变化是否超过阈值
        if (!last_published_point_ || 
            isSignificantChange(filtered_point, *last_published_point_)) {
            
            auto target_pose = geometry_msgs::msg::PoseStamped();
            
            // 设置时间戳和坐标系
            target_pose.header.stamp = this->now();
            target_pose.header.frame_id = "world";

            // 设置位置
            target_pose.pose.position = filtered_point;

            // 设置姿态
            target_pose.pose.orientation.w = 0.70711;
            target_pose.pose.orientation.x = -1.2504e-06;
            target_pose.pose.orientation.y = -1.2543e-06;
            target_pose.pose.orientation.z = 0.70711;

            // 发布目标位姿
            publisher_->publish(target_pose);
            
            RCLCPP_INFO(this->get_logger(), "发布新的目标位姿: x=%.3f, y=%.3f, z=%.3f",
                filtered_point.x, filtered_point.y, filtered_point.z);

            // 更新上次发布的点
            if (!last_published_point_) {
                last_published_point_ = std::make_unique<geometry_msgs::msg::Point>();
            }
            *last_published_point_ = filtered_point;
        }
    }

    bool isSignificantChange(const geometry_msgs::msg::Point& p1, 
                           const geometry_msgs::msg::Point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        return distance > position_threshold_;
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::deque<geometry_msgs::msg::Point> point_buffer_;  // 点的缓存队列
    std::unique_ptr<geometry_msgs::msg::Point> last_published_point_;  // 上次发布的点
    int filter_window_size_;  // 滤波窗口大小
    double position_threshold_;  // 位置变化阈值
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MatchPointConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 