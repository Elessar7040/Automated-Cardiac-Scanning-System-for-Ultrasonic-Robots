#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_ros import TransformException
from sensor_msgs_py import point_cloud2
import numpy as np
from transforms3d.quaternions import quat2mat
import struct

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('point_cloud_transformer')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/custom_ns/custom_camera/custom_points',
            self.point_cloud_callback,
            10)
            
        self.publisher = self.create_publisher(
            PointCloud2,
            '/transformed_points',
            10)

    def transform_to_matrix(self, transform):
        """将transform转换为4x4变换矩阵"""
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # 创建旋转矩阵
        rot_matrix = quat2mat([rot.w, rot.x, rot.y, rot.z])
        
        # 创建4x4变换矩阵
        matrix = np.eye(4)
        matrix[:3, :3] = rot_matrix
        matrix[:3, 3] = [trans.x, trans.y, trans.z]
        return matrix

    def extract_xyz_from_points(self, cloud_points):
        """从点云数据中提取XYZ坐标"""
        xyz_points = []
        for point in cloud_points:
            # 确保我们得到浮点数
            x = float(point[0])
            y = float(point[1])
            z = float(point[2])
            xyz_points.append([x, y, z])
        return np.array(xyz_points, dtype=np.float64)

    def point_cloud_callback(self, msg):
        try:
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                'world',
                msg.header.frame_id,
                rclpy.time.Time())
            
            # 获取变换矩阵
            transform_matrix = self.transform_to_matrix(transform)
            
            # 读取点云数据
            pc_points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z")))
            if not pc_points:
                self.get_logger().warn('Received empty point cloud')
                return
                
            # 提取并转换XYZ坐标
            points = self.extract_xyz_from_points(pc_points)
            
            # 添加齐次坐标
            points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
            
            # 应用变换
            transformed_points = np.dot(points_homogeneous, transform_matrix.T)[:, :3]
            
            # 创建新的点云消息
            transformed_cloud = point_cloud2.create_cloud_xyz32(
                header=msg.header,
                points=transformed_points.tolist()
            )
            transformed_cloud.header.frame_id = 'world'
            transformed_cloud.header.stamp = msg.header.stamp
            
            # 发布转换后的点云
            self.publisher.publish(transformed_cloud)
            
            self.get_logger().info(f'Transformed point cloud with {len(transformed_points)} points')
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform point cloud: {ex}')
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
            if 'points' in locals():
                self.get_logger().error(f'Points shape: {points.shape}')
                self.get_logger().error(f'Points dtype: {points.dtype}')

def main():
    rclpy.init()
    node = PointCloudTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()