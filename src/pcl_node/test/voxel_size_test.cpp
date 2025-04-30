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
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <vector>
#include <chrono>

class VoxelBenchmark : public rclcpp::Node 
{
public:
    explicit VoxelBenchmark() : Node("voxel_benchmark"), received_cloud_(false)
    {
        // 初始化参数
        this->declare_parameter("start_voxel_size", 0.01);
        this->declare_parameter("end_voxel_size", 0.1);
        this->declare_parameter("step_size", 0.01);
        this->declare_parameter("result_file", "/home/elessar/russ_ws/ws7/src/pcl_node/test_results/voxel_benchmark_results.csv");
        
        start_voxel_size_ = this->get_parameter("start_voxel_size").as_double();
        end_voxel_size_ = this->get_parameter("end_voxel_size").as_double();
        step_size_ = this->get_parameter("step_size").as_double();
        result_file_ = this->get_parameter("result_file").as_string();
        
        // 确保目录存在
        std::string dir_path = "/home/elessar/russ_ws/ws7/src/pcl_node/test_results/";
        if (system(("mkdir -p " + dir_path).c_str()) != 0) {
            RCLCPP_WARN(this->get_logger(), "创建结果目录失败: %s", dir_path.c_str());
        }
        
        // 初始化结果文件
        std::ofstream outfile(result_file_);
        if (outfile.is_open()) {
            outfile << "体素大小,点云数量,处理时间(ms)" << std::endl;
            outfile.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法创建结果文件: %s", result_file_.c_str());
        }
        
        // 初始化TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建发布者
        cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "transformed_points", 10);
            
        // 创建订阅者
        cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/custom_ns/custom_camera/custom_points", 10,
            std::bind(&VoxelBenchmark::cloudCallback, this, std::placeholders::_1));
            
        // 创建定时器用于定期检查是否接收到点云并进行基准测试
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&VoxelBenchmark::timerCallback, this));
            
        RCLCPP_INFO(this->get_logger(), "体素基准测试节点已初始化，等待接收点云...");
        RCLCPP_INFO(this->get_logger(), "体素大小范围: %.2f 到 %.2f，步长: %.2f", 
                   start_voxel_size_, end_voxel_size_, step_size_);
    }
    
private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
        // 只保存第一个接收到的点云用于基准测试
        if (!received_cloud_) {
            input_cloud_ = input_cloud;
            received_cloud_ = true;
            RCLCPP_INFO(this->get_logger(), "接收到点云数据，准备进行基准测试...");
        }
    }
    
    void timerCallback()
    {
        // 检查是否已接收到点云
        if (!received_cloud_) {
            RCLCPP_INFO(this->get_logger(), "等待接收点云数据...");
            return;
        }
        
        // 如果已经完成测试，则停止定时器
        if (already_benchmarked_) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "开始体素滤波基准测试...");
        performBenchmark();
        already_benchmarked_ = true;
        RCLCPP_INFO(this->get_logger(), "基准测试完成，结果已保存到: %s", result_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "可以使用单独的Python脚本生成图表");
    }
    
    void performBenchmark()
    {
        // 准备结果容器
        std::vector<double> voxel_sizes;
        std::vector<size_t> point_counts;
        std::vector<double> processing_times;
        
        // 将ROS消息转换为PCL点云（只需转换一次）
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud_, *pcl_cloud);
        
        try {
            // 获取从相机到世界坐标系的变换
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                "table_base_link",                  // 目标坐标系
                input_cloud_->header.frame_id,      // 源坐标系
                input_cloud_->header.stamp);        // 使用点云的时间戳
                
            // 创建变换矩阵
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            
            // 添加旋转
            Eigen::AngleAxisf rotation_y(M_PI / 2, Eigen::Vector3f::UnitY());
            transform.rotate(rotation_y);
            
            Eigen::AngleAxisf rotation_z(M_PI / 3, Eigen::Vector3f::UnitZ());
            transform.rotate(rotation_z);
            
            Eigen::AngleAxisf rotation_x(M_PI / 6, Eigen::Vector3f::UnitX());
            transform.rotate(rotation_x);
            
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
            
            // 执行点云变换（只需变换一次）
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);
            
            RCLCPP_INFO(this->get_logger(), "原始点云大小: %zu 点", transformed_cloud->size());
            
            // 针对不同的体素大小进行测试
            for (double voxel_size = start_voxel_size_; 
                 voxel_size <= end_voxel_size_ + 0.0001; // 加一个小数字避免浮点数比较问题
                 voxel_size += step_size_) {
                
                // 创建体素滤波器
                pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
                pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                voxel_filter.setInputCloud(transformed_cloud);
                voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
                
                // 记录开始时间
                auto start_time = std::chrono::high_resolution_clock::now();
                
                // 执行滤波
                voxel_filter.filter(*filtered_cloud);
                
                // 记录结束时间
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                
                // 保存结果
                voxel_sizes.push_back(voxel_size);
                point_counts.push_back(filtered_cloud->size());
                processing_times.push_back(duration);
                
                RCLCPP_INFO(this->get_logger(), 
                           "体素大小: %.2f, 点云数量: %zu, 处理时间: %ld ms", 
                           voxel_size, filtered_cloud->size(), duration);
                
                // 发布最新的点云（仅供可视化）
                sensor_msgs::msg::PointCloud2 output_cloud;
                pcl::toROSMsg(*filtered_cloud, output_cloud);
                output_cloud.header.frame_id = "world";
                output_cloud.header.stamp = input_cloud_->header.stamp;
                cloud_publisher_->publish(output_cloud);
                
                // 等待一小段时间，让RViz有机会更新显示
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            // 将结果写入文件
            std::ofstream outfile(result_file_);
            if (outfile.is_open()) {
                outfile << "体素大小,点云数量,处理时间(ms)" << std::endl;
                for (size_t i = 0; i < voxel_sizes.size(); ++i) {
                    outfile << voxel_sizes[i] << "," 
                           << point_counts[i] << "," 
                           << processing_times[i] << std::endl;
                }
                outfile.close();
                RCLCPP_INFO(this->get_logger(), "结果已保存到: %s", result_file_.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "无法写入结果文件: %s", result_file_.c_str());
            }
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "无法转换点云: %s", ex.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理点云时发生错误: %s", e.what());
        }
    }

    // TF2相关成员
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS发布者和订阅者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 点云数据
    sensor_msgs::msg::PointCloud2::SharedPtr input_cloud_;
    bool received_cloud_ = false;
    bool already_benchmarked_ = false;

    // 参数
    double start_voxel_size_;
    double end_voxel_size_;
    double step_size_;
    std::string result_file_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoxelBenchmark>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}