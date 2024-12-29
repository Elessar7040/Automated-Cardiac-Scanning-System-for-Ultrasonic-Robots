#include <rclcpp/rclcpp.hpp>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// 添加引用
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // build the filter
    sor.setInputCloud(cloudPtr);
    // 通过 lx, ly, lz 分别设置体素栅格在 XYZ 3个方向上的尺寸
    sor.setLeafSize(0.5, 0.5, 0.5);
    // apply filter
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 cloud_vog;
    pcl_conversions::moveFromPCL(cloud_filtered, cloud_vog);

    // Publish the data
    pub.publish(cloud_vog);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pointcloud_filter_pub");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud 输入
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_pcd", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud 输出
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_pcd/filter", 1);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}