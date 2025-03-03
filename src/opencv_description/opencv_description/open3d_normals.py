#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            PointCloud2,
            'transformed_points',
            self.cloud_callback,
            10)
        
        self.saved = False

    def cloud_callback(self, msg):
        if self.saved:
            return
            
        # 将ROS点云消息转换为numpy数组
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        points = np.array(points)
        
        # 创建open3d点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 计算法向量
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        # 保存点云
        o3d.io.write_point_cloud("src/opencv_description/pointcloud_output/cloud_with_normals.pcd", pcd)
        
        # 创建可视化窗口
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Point Cloud with Normals", width=800, height=600)
        
        # 添加点云到可视化器
        vis.add_geometry(pcd)
        
        # 设置渲染选项
        opt = vis.get_render_option()
        opt.point_size = 2.0
        opt.point_show_normal = True
        
        # 设置默认视角
        ctr = vis.get_view_control()
        ctr.set_front([0, 1, 1])
        ctr.set_lookat([0, 0, 0])
        ctr.set_up([0, -1, 0])
        ctr.set_zoom(0.8)
        
        # 更新可视化
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        
        # 捕获图像并保存
        image = vis.capture_screen_float_buffer(do_render=True)
        image_array = np.asarray(image)
        image_array = (image_array * 255).astype(np.uint8)
        
        # 创建Open3D图像对象
        o3d_image = o3d.geometry.Image(image_array)
        
        # 使用Open3D的io模块保存图像
        o3d.io.write_image("src/opencv_description/pointcloud_output/normals_visualization.png", o3d_image)
        
        # 显示可视化窗口
        vis.run()
        
        # 关闭可视化器
        vis.destroy_window()
        
        self.saved = True
        self.get_logger().info('点云和可视化图像已保存')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()