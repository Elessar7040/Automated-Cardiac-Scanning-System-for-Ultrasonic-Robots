#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>

/*
 * 对于每个搜索半径:
 * 使用当前半径计算法向量
 * 记录计算时间
 * 
 * 计算与参考法向量的相似度指标:
 * 余弦相似度（法向量方向的一致性）
 * 平均角度差异（以度为单位）
 * 法向量一致率（角度差异小于30度的百分比）
 * 将结果保存到CSV文件
 * 
 * 这个程序提供了三个精度指标：
 * 余弦相似度：值越接近1表示法向量方向越一致
 * 平均角度差异：以度为单位，值越小表示法向量方向越接近
 * 法向量一致率：角度差异小于30度的百分比，值越高表示估计结果越准确
*/

class NormalRadiusBenchmark : public rclcpp::Node
{
public:
    explicit NormalRadiusBenchmark() : Node("normal_radius_benchmark")
    {
        // 初始化参数
        this->declare_parameter("cloud_path", "/home/elessar/russ_ws/ws7/src/pcl_node/point_output/filtered_cloud.pcd");
        this->declare_parameter("start_radius", 0.01);
        this->declare_parameter("end_radius", 0.2);
        this->declare_parameter("step_size", 0.01);
        this->declare_parameter("reference_radius", 0.2); // 用于计算精度参考的半径
        this->declare_parameter("result_file", "/home/elessar/russ_ws/ws7/src/pcl_node/test_results/normal_radius_benchmark_results.csv");
        this->declare_parameter("visualize", false); // 是否可视化

        cloud_path_ = this->get_parameter("cloud_path").as_string();
        start_radius_ = this->get_parameter("start_radius").as_double();
        end_radius_ = this->get_parameter("end_radius").as_double();
        step_size_ = this->get_parameter("step_size").as_double();
        reference_radius_ = this->get_parameter("reference_radius").as_double();
        result_file_ = this->get_parameter("result_file").as_string();
        visualize_ = this->get_parameter("visualize").as_bool();

        // 确保目录存在
        std::string dir_path = "/home/elessar/russ_ws/ws7/src/pcl_node/test_results/";
        if (system(("mkdir -p " + dir_path).c_str()) != 0)
        {
            RCLCPP_WARN(this->get_logger(), "创建结果目录失败: %s", dir_path.c_str());
        }

        // 初始化结果文件
        std::ofstream outfile(result_file_);
        if (outfile.is_open())
        {
            outfile << "搜索半径,法向量数量,处理时间(ms),平均余弦相似度,平均角度差异(度),法向量一致率(%)" << std::endl;
            outfile.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建结果文件: %s", result_file_.c_str());
        }

        // 创建发布者
        cloud_with_normals_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "cloud_with_normals", 10);

        // 创建定时器来启动基准测试
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&NormalRadiusBenchmark::performBenchmark, this));

        RCLCPP_INFO(this->get_logger(), "法向量搜索半径基准测试节点已初始化");
        RCLCPP_INFO(this->get_logger(), "搜索半径范围: %.2f 到 %.2f，步长: %.2f",
                    start_radius_, end_radius_, step_size_);
        RCLCPP_INFO(this->get_logger(), "参考搜索半径：%.2f", reference_radius_);
    }

private:
    void performBenchmark()
    {
        // 只运行一次
        timer_->cancel();

        try
        {
            // 加载点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path_, *cloud) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "无法加载点云文件: %s", cloud_path_.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "加载点云，包含 %zu 个点", cloud->size());

            // 存储不同半径的测试结果
            std::vector<double> radius_values;
            std::vector<size_t> normal_counts;
            std::vector<double> processing_times;
            std::vector<double> cosine_similarities;
            std::vector<double> angle_differences;
            std::vector<double> consistency_rates;

            // 首先使用参考半径计算法向量，作为"标准参考"
            RCLCPP_INFO(this->get_logger(), "计算参考法向量（半径 = %.2f）...", reference_radius_);

            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_ref;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ref(new pcl::search::KdTree<pcl::PointXYZ>());
            pcl::PointCloud<pcl::Normal>::Ptr reference_normals(new pcl::PointCloud<pcl::Normal>);

            ne_ref.setInputCloud(cloud);
            ne_ref.setSearchMethod(tree_ref);
            ne_ref.setRadiusSearch(reference_radius_);
            ne_ref.compute(*reference_normals);

            RCLCPP_INFO(this->get_logger(), "计算了 %zu 个参考法向量", reference_normals->size());

            // 针对不同搜索半径测试法向量估计
            for (double radius = start_radius_;
                 radius <= end_radius_ + 0.0001; // 加一个小数字避免浮点数比较问题
                 radius += step_size_)
            {

                // 计算法向量
                pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

                ne.setInputCloud(cloud);
                ne.setSearchMethod(tree);
                ne.setRadiusSearch(radius);

                // 记录开始时间
                auto start_time = std::chrono::high_resolution_clock::now();

                // 执行法向量计算
                ne.compute(*normals);

                // 记录结束时间
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                // 计算与参考法向量的相似度（余弦相似度）
                double total_cosine_similarity = 0.0;
                double total_angle_diff = 0.0;
                int consistent_count = 0;
                size_t valid_count = 0;

                for (size_t i = 0; i < normals->size(); ++i)
                {
                    // 检查法向量是否有效
                    if (std::isfinite(normals->points[i].normal_x) &&
                        std::isfinite(normals->points[i].normal_y) &&
                        std::isfinite(normals->points[i].normal_z) &&
                        std::isfinite(reference_normals->points[i].normal_x) &&
                        std::isfinite(reference_normals->points[i].normal_y) &&
                        std::isfinite(reference_normals->points[i].normal_z))
                    {

                        Eigen::Vector3f normal1(normals->points[i].normal_x,
                                                normals->points[i].normal_y,
                                                normals->points[i].normal_z);
                        Eigen::Vector3f normal2(reference_normals->points[i].normal_x,
                                                reference_normals->points[i].normal_y,
                                                reference_normals->points[i].normal_z);

                        // 确保法向量是单位向量
                        normal1.normalize();
                        normal2.normalize();

                        // 计算余弦相似度（点积）
                        float dot_product = std::abs(normal1.dot(normal2)); // 取绝对值，因为法向量方向可能相反
                        total_cosine_similarity += dot_product;

                        // 计算角度差异（弧度）
                        float angle_rad = std::acos(std::min(1.0f, std::max(-1.0f, dot_product)));
                        total_angle_diff += angle_rad * 180.0 / M_PI; // 转换为角度

                        // 计算"一致性"，即角度差异小于30度的比例
                        if (angle_rad < M_PI / 6.0)
                        { // 30度 = π/6 弧度
                            consistent_count++;
                        }

                        valid_count++;
                    }
                }

                // 计算平均值
                double avg_cosine_similarity = valid_count > 0 ? total_cosine_similarity / valid_count : 0.0;
                double avg_angle_diff = valid_count > 0 ? total_angle_diff / valid_count : 0.0;
                double consistency_rate = valid_count > 0 ? (consistent_count * 100.0) / valid_count : 0.0;

                // 保存结果
                radius_values.push_back(radius);
                normal_counts.push_back(normals->size());
                processing_times.push_back(duration);
                cosine_similarities.push_back(avg_cosine_similarity);
                angle_differences.push_back(avg_angle_diff);
                consistency_rates.push_back(consistency_rate);

                RCLCPP_INFO(this->get_logger(),
                            "搜索半径: %.2f, 法向量数量: %zu, 处理时间: %ld ms, 余弦相似度: %.4f, 角度差异: %.2f度, 一致率: %.2f%%",
                            radius, normals->size(), duration, avg_cosine_similarity, avg_angle_diff, consistency_rate);

                // 创建并发布带法向量的点云（用于可视化）
                if (visualize_)
                {
                    // 合并点和法向量
                    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
                    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

                    // 发布点云法向量
                    sensor_msgs::msg::PointCloud2 output_cloud;
                    pcl::toROSMsg(*cloud_with_normals, output_cloud);
                    output_cloud.header.frame_id = "world";
                    output_cloud.header.stamp = this->now();
                    cloud_with_normals_publisher_->publish(output_cloud);

                    // 可视化
                    visualizePointCloudWithNormals(cloud, normals);

                    // 等待一小段时间，让用户观察
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }
            }

            // 将结果写入文件
            std::ofstream outfile(result_file_);
            if (outfile.is_open())
            {
                outfile << "搜索半径,法向量数量,处理时间(ms),平均余弦相似度,平均角度差异(度),法向量一致率(%)" << std::endl;
                for (size_t i = 0; i < radius_values.size(); ++i)
                {
                    outfile << radius_values[i] << ","
                            << normal_counts[i] << ","
                            << processing_times[i] << ","
                            << cosine_similarities[i] << ","
                            << angle_differences[i] << ","
                            << consistency_rates[i] << std::endl;
                }
                outfile.close();
                RCLCPP_INFO(this->get_logger(), "结果已保存到: %s", result_file_.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "无法写入结果文件: %s", result_file_.c_str());
            }

            RCLCPP_INFO(this->get_logger(), "法向量搜索半径基准测试完成");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "处理点云时发生错误: %s", e.what());
        }
    }

    void visualizePointCloudWithNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
        // 在新线程中运行可视化，避免阻塞ROS节点
        std::thread vis_thread([cloud, normals]()
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
            
            // 显示可视化窗口，但只显示3秒
            auto start_time = std::chrono::high_resolution_clock::now();
            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                auto current_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                if (duration > 3) {
                    break;
                }
            }
            viewer->close(); });

        // 分离线程
        vis_thread.detach();
    }

    // 参数
    std::string cloud_path_;
    double start_radius_;
    double end_radius_;
    double step_size_;
    double reference_radius_;
    std::string result_file_;
    bool visualize_;

    // ROS发布者和定时器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_with_normals_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NormalRadiusBenchmark>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}