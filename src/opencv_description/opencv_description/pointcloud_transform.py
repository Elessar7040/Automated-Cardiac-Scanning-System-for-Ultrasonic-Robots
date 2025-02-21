import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_sensor_msgs

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('point_cloud_transformer')
        
        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 订阅原始点云
        self.subscription = self.create_subscription(
            PointCloud2,
            '/custom_ns/custom_camera/custom_points',  # 原始点云话题
            self.point_cloud_callback,
            10)
            
        # 发布转换后的点云
        self.publisher = self.create_publisher(
            PointCloud2,
            '/transformed_points',  # 转换后的点云话题
            10)

    def point_cloud_callback(self, msg):
        try:
            # 等待转换关系可用
            transform = self.tf_buffer.lookup_transform(
                'world',           # 目标坐标系
                msg.header.frame_id,  # 源坐标系
                rclpy.time.Time())
            
            # 转换点云
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(msg, transform)
            
            # 发布转换后的点云
            self.publisher.publish(transformed_cloud)
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform point cloud: {ex}')

def main():
    rclpy.init()
    node = PointCloudTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()