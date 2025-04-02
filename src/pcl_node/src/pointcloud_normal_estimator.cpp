#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>
#include <memory>
#include <Eigen/Geometry> // 添加Eigen几何库，用于四元数计算

class PointCloudNormalEstimator : public rclcpp::Node
{
public:
    explicit PointCloudNormalEstimator() : Node("pointcloud_normal_estimator")
    {
        // 初始化参数
        this->declare_parameter("cloud_path", "/home/elessar/russ_ws/ws7/src/pcl_node/point_output/filtered_cloud.pcd"); // 点云保存路径
        this->declare_parameter("normal_radius", 0.04); // 法向量估计的搜索半径
        this->declare_parameter("visualize", true);     // 是否可视化

        // 添加查询点参数
        this->declare_parameter("query_point_x", 0.066);
        this->declare_parameter("query_point_y", 0.79);
        this->declare_parameter("query_point_z", 0.79);
        this->declare_parameter("query_radius", 0.1);

        cloud_path_ = this->get_parameter("cloud_path").as_string();
        normal_radius_ = this->get_parameter("normal_radius").as_double();
        visualize_ = this->get_parameter("visualize").as_bool();

        // 创建发布者
        cloud_with_normals_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "point_cloud_with_normals", 10);

        // 创建定时器，加载点云并计算法向量
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PointCloudNormalEstimator::processCloud, this));

        // 创建定时任务，延迟执行查询
        query_timer_ = this->create_wall_timer(
            std::chrono::seconds(3), // 延迟3秒，确保点云已加载
            std::bind(&PointCloudNormalEstimator::performQueryIfRequested, this));

        RCLCPP_INFO(this->get_logger(), "PointCloud normal estimator initialized");
        RCLCPP_INFO(this->get_logger(), "Will load cloud from: %s", cloud_path_.c_str());
    }

    // 根据世界坐标查询法向量
    Eigen::Vector3f getNormalAtPosition(const Eigen::Vector3f &position, float radius = 0.01)
    {
        if (!cloud_with_normals_)
        {
            RCLCPP_WARN(this->get_logger(), "Cloud with normals not available yet");
            return Eigen::Vector3f(0, 0, 0);
        }

        // 在点云中寻找离查询点最近的点
        pcl::PointNormal nearest_point;
        float min_distance = std::numeric_limits<float>::max();

        for (const auto &point : *cloud_with_normals_)
        {
            Eigen::Vector3f point_pos(point.x, point.y, point.z);
            float distance = (point_pos - position).norm();

            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_point = point;
            }
        }

        if (min_distance > radius)
        {
            RCLCPP_WARN(this->get_logger(), "No point found within radius %f at position (%f, %f, %f)",
                        radius, position.x(), position.y(), position.z());
            return Eigen::Vector3f(0, 0, 0);
        }

        return Eigen::Vector3f(nearest_point.normal_x, nearest_point.normal_y, nearest_point.normal_z);
    }

    // 根据法向量计算四元数
    Eigen::Quaternionf getQuaternionFromNormal(const Eigen::Vector3f &normal)
    {
        // 确保法向量为单位向量
        Eigen::Vector3f normalized_normal = normal.normalized();
        
        // 假设法向量指向z轴正方向
        Eigen::Vector3f z_axis(0, 0, 1);
        
        // 计算旋转轴（法向量与z轴的叉积）
        Eigen::Vector3f rotation_axis = z_axis.cross(normalized_normal);
        
        // 如果法向量与z轴平行，需要特殊处理
        if (rotation_axis.norm() < 1e-6) {
            // 如果法向量指向z轴正方向，则不需要旋转
            if (normalized_normal.dot(z_axis) > 0) {
                return Eigen::Quaternionf::Identity();
            } else {
                // 如果法向量指向z轴负方向，则旋转180度
                return Eigen::Quaternionf(0, 1, 0, 0); // 绕x轴旋转180度
            }
        }
        
        // 计算旋转角度（法向量与z轴的夹角）
        float angle = std::acos(normalized_normal.dot(z_axis));
        
        // 使用旋转轴和角度创建四元数
        rotation_axis.normalize();
        return Eigen::Quaternionf(Eigen::AngleAxisf(angle, rotation_axis));
    }

    // 获取点位的法向量和对应的四元数
    std::pair<Eigen::Vector3f, Eigen::Quaternionf> getNormalAndQuaternion(
        const Eigen::Vector3f &position, float radius = 0.01)
    {
        Eigen::Vector3f normal = getNormalAtPosition(position, radius);
        Eigen::Quaternionf quaternion = getQuaternionFromNormal(normal);
        return {normal, quaternion};
    }

private:
    void processCloud()
    {
        // 只处理一次
        timer_->cancel();

        try
        {
            // 加载点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path_, *cloud) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud from %s", cloud_path_.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Loaded cloud with %zu points", cloud->size());

            // 计算法向量
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(normal_radius_);
            ne.compute(*normals);

            RCLCPP_INFO(this->get_logger(), "Computed %zu normals", normals->size());

            // 合并点和法向量
            cloud_with_normals_.reset(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*cloud, *normals, *cloud_with_normals_);

            // 发布点云法向量
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl::toROSMsg(*cloud_with_normals_, output_cloud);
            output_cloud.header.frame_id = "world";
            output_cloud.header.stamp = this->now();
            cloud_with_normals_publisher_->publish(output_cloud);

            // 可视化
            if (visualize_)
            {
                visualizePointCloudWithNormals(cloud, normals);
            }

            RCLCPP_INFO(this->get_logger(), "Point cloud with normals processed and published");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        }
    }

    void visualizePointCloudWithNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
        // 在新线程中运行可视化，避免阻塞ROS节点
        std::thread vis_thread([this, cloud, normals]()
                               {
            // 创建可视化器
            pcl::visualization::PCLVisualizer::Ptr viewer(
                new pcl::visualization::PCLVisualizer("Point Cloud with Normals"));
            viewer->setBackgroundColor(0, 0, 0);
            
            // 添加点云
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
                color_handler(cloud, 255, 255, 255);
            viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");
            
            // 添加法向量
            viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
                cloud, normals, 10, 0.05, "normals");
                
            // 设置相机位置
            viewer->initCameraParameters();
            
            // 显示可视化窗口
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } });

        // 分离线程
        vis_thread.detach();
    }

    void performQueryIfRequested()
    {
        // 取消定时器，只执行一次
        query_timer_->cancel();

        // 检查是否已加载点云
        if (!cloud_with_normals_ || cloud_with_normals_->empty())
        {
            RCLCPP_WARN(this->get_logger(), "点云尚未加载完成，无法查询法向量");
            return;
        }

        // 获取查询点参数
        double x = this->get_parameter("query_point_x").as_double();
        double y = this->get_parameter("query_point_y").as_double();
        double z = this->get_parameter("query_point_z").as_double();
        double radius = this->get_parameter("query_radius").as_double();

        // 查询法向量和四元数
        Eigen::Vector3f position(x, y, z);
        auto [normal, quaternion] = getNormalAndQuaternion(position, radius);

        // 输出法向量
        RCLCPP_INFO(this->get_logger(),
                    "点 (%.3f, %.3f, %.3f) 的法向量为 (%.3f, %.3f, %.3f)",
                    x, y, z, normal.x(), normal.y(), normal.z());
                    
        // 输出四元数
        RCLCPP_INFO(this->get_logger(),
                    "对应的四元数为 w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                    quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
                    
        // 输出旋转矩阵
        Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();
        RCLCPP_INFO(this->get_logger(), "对应的旋转矩阵为:");
        RCLCPP_INFO(this->get_logger(), "[%.3f, %.3f, %.3f]", 
                   rotation_matrix(0,0), rotation_matrix(0,1), rotation_matrix(0,2));
        RCLCPP_INFO(this->get_logger(), "[%.3f, %.3f, %.3f]", 
                   rotation_matrix(1,0), rotation_matrix(1,1), rotation_matrix(1,2));
        RCLCPP_INFO(this->get_logger(), "[%.3f, %.3f, %.3f]", 
                   rotation_matrix(2,0), rotation_matrix(2,1), rotation_matrix(2,2));
    }

    std::string cloud_path_;
    double normal_radius_;
    bool visualize_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_with_normals_publisher_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_;
    rclcpp::TimerBase::SharedPtr query_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudNormalEstimator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}