#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

class PCLSubscriberNode : public rclcpp::Node {
public:
    PCLSubscriberNode();
    ~PCLSubscriberNode();

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    std::thread viewer_thread_;
};

PCLSubscriberNode::PCLSubscriberNode()
    : Node("pcl_subscriber_node"),
      viewer_(new pcl::visualization::PCLVisualizer("3D Viewer")) {
    // 初始化订阅者
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/custom_ns/custom_camera/custom_points",  // 替换为实际点云话题名
        10,
        std::bind(&PCLSubscriberNode::pointcloudCallback, this, std::placeholders::_1)
    );

    // 配置 PCL 可视化器
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();

    // 启动可视化线程
    viewer_thread_ = std::thread([this]() {
        while (!viewer_->wasStopped()) {
            viewer_->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    RCLCPP_INFO(this->get_logger(), "PCL Subscriber Node has been started.");
}

PCLSubscriberNode::~PCLSubscriberNode() {
    viewer_->close();
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void PCLSubscriberNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // 将 ROS2 的 PointCloud2 转换为 PCL 格式
    pcl::fromROSMsg(*msg, *cloud);

    // 更新 PCL Viewer 的点云
    if (!viewer_->updatePointCloud(cloud, "sample cloud")) {
        viewer_->addPointCloud(cloud, "sample cloud");
    }

    RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points.", cloud->points.size());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    

