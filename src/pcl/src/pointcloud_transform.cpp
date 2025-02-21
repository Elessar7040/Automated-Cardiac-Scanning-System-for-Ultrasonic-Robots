#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>

class PointCloudTransformer : public rclcpp::Node 
{
public:
    explicit PointCloudTransformer() : Node("pointcloud_transformer") 
    {
        // 初始化参数
        this->declare_parameter("voxel_size", 0.05);  // 默认体素大小为1cm
        voxel_size_ = this->get_parameter("voxel_size").as_double();

        // 初始化TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建发布者
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "transformed_points", 10);

        // 创建订阅者
        cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/custom_ns/custom_camera/custom_points", 10,
            std::bind(&PointCloudTransformer::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PointCloud transformer initialized with voxel size: %f", voxel_size_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
        try 
        {
            // 将ROS消息转换为PCL点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*input_cloud, *pcl_cloud);

            // 获取从相机到世界坐标系的变换
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "base_link",                    // 目标坐标系
                input_cloud->header.frame_id, // 源坐标系
                input_cloud->header.stamp);   // 使用点云的时间戳

            // 创建变换矩阵
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();

            // 设置平移
            transform.translation() << 
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z;

            // 设置旋转（使用四元数）
            Eigen::Quaternionf q(
                transform_stamped.transform.rotation.w,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z);
            transform.rotate(q);

            // 执行点云变换
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);

            // 应用体素滤波
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.setInputCloud(transformed_cloud);
            voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            voxel_filter.filter(*filtered_cloud);

            // 转换回ROS消息
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl::toROSMsg(*filtered_cloud, output_cloud);
            output_cloud.header.frame_id = "world";
            output_cloud.header.stamp = input_cloud->header.stamp;

            // 发布转换后的点云
            cloud_publisher_->publish(output_cloud);

            RCLCPP_DEBUG(this->get_logger(), 
                "Point cloud transformed and filtered: original size=%zu, filtered size=%zu",
                pcl_cloud->size(), filtered_cloud->size());
        }
        catch (const tf2::TransformException& ex) 
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        }
    }

    // TF2相关成员
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS发布者和订阅者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

    // 体素滤波参数
    double voxel_size_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}