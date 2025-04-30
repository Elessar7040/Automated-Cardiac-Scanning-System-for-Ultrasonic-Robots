#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>

/*
 * 可视化点云和法向量
 *
 * 参数:
 * - cloud_path: 点云文件路径
 * - normal_radius: 法向量搜索半径
 *
 * 使用方法:
 * 1. 编译
 * cd /home/elessar/russ_ws/ws7/install/pcl_node/lib/pcl_node
 * 2. 运行
 * ./view_point_cloud_normals [点云文件路径] [法向量搜索半径]
 * ./view_point_cloud_normals /home/elessar/russ_ws/ws7/src/pcl_node/point_output/filtered_cloud.pcd 0.1
 * 3. 按 'q' 退出可视化窗口
 */

int main(int argc, char **argv)
{
    // 解析命令行参数
    std::string cloud_path = "/home/elessar/russ_ws/ws7/src/pcl_node/point_output/filtered_cloud.pcd";
    double normal_radius = 0.1;

    if (argc > 1)
    {
        cloud_path = argv[1];
    }

    if (argc > 2)
    {
        normal_radius = std::stod(argv[2]);
    }

    std::cout << "加载点云文件: " << cloud_path << std::endl;
    std::cout << "法向量搜索半径: " << normal_radius << std::endl;

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(cloud_path, *cloud) == -1)
    {
        std::cerr << "无法加载点云文件: " << cloud_path << std::endl;
        return -1;
    }

    std::cout << "加载了 " << cloud->size() << " 个点" << std::endl;

    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_radius);

    std::cout << "计算法向量中..." << std::endl;
    ne.compute(*normals);
    std::cout << "计算了 " << normals->size() << " 个法向量" << std::endl;

    // 设置法向量的视点
    ne.setViewPoint(0.0, 0.0, 0.0); // 设置视点为原点

    // 可视化点云和法向量
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("点云法向量可视化"));
    viewer->setBackgroundColor(255, 255, 255);

    // 添加点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, "cloud");

    // 添加法向量，参数分别是：法向量间隔(每10个点显示一个法向量)和法向量长度(0.05)
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 1, 0.05, "normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "normals");

    // 添加坐标系
    viewer->addCoordinateSystem(0.1);

    // 设置相机参数
    viewer->initCameraParameters();

    std::cout << "按 'q' 退出可视化窗口" << std::endl;

    // 循环直到窗口关闭
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}