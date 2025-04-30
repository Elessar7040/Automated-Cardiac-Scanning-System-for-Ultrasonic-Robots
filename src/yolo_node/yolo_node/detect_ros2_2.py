#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os

'''
用于ur_group图像跟踪测试
'''

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 创建订阅者，接收图像
        self.subscription = self.create_subscription(
            Image,
            '/custom_ns/custom_camera/custom_image',  # 根据实际的话题名修改
            self.image_callback,
            1)
        
        # 创建控制订阅者
        self.control_subscription = self.create_subscription(
            Bool,
            '/yolo/start_detection',
            self.control_callback,
            1)
        
        # 初始化发布者和订阅者
        self.point_msg = Point()
        self.point_msg.x = 320.0
        self.point_msg.y = 240.0
        self.point_msg.z = 0.0
        
        # 创建发布者，发布处理后的图像
        self.image_publisher = self.create_publisher(
            Image,
            '/yolo/detected_image',
            1)
        
        # 创建发布者，发布目标中心坐标
        self.target_publisher = self.create_publisher(
            Point,
            '/yolo/target_center',
            10)
            
        # 添加处理状态发布者
        self.status_publisher = self.create_publisher(
            Bool,
            '/yolo/processing_status',
            1)
        
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp4/weights/best.pt')
        
        # 检测开关
        self.detection_active = False
        
        self.get_logger().info('YOLO检测节点已启动，等待控制信号')

    def control_callback(self, msg):
        """控制回调函数，接收到True时启动图像处理，False时停止"""
        if msg.data and not self.detection_active:
            self.start_detection()
        elif not msg.data and self.detection_active:
            self.stop_detection()
    
    def start_detection(self):
        """启动图像处理"""
        self.detection_active = True
        
        # 发布处理开始状态
        status_msg = Bool()
        status_msg.data = True
        self.status_publisher.publish(status_msg)
        
        self.get_logger().info('开始图像处理')
    
    def stop_detection(self):
        """停止图像处理"""
        self.detection_active = False
        
        # 发布处理结束状态
        status_msg = Bool()
        status_msg.data = False
        self.status_publisher.publish(status_msg)
        
        self.get_logger().info('停止图像处理')

    def image_callback(self, msg):
        # 如果检测未激活，直接返回
        if not self.detection_active:
            return
            
        # 将ROS图像消息转换为OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 运行YOLO检测
        results = self.model(cv_image)
        
        # 在图像上绘制检测结果
        annotated_frame = results[0].plot()
        
        # 发布检测到的目标中心坐标
        if len(results[0].obb.cls) > 0:
            # 获取第一个检测到的目标（假设只关注一个目标）
            x1, y1, x2, y2 = results[0].obb.xyxy[0].tolist()  # 获取边界框坐标
            
            # 计算中心点
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # 创建并发布Point消息
            point_msg = Point()
            point_msg.x = float(center_x)
            point_msg.y = float(center_y)
            point_msg.z = 0.0  # 2D图像中z坐标设为0
            
            self.target_publisher.publish(point_msg)
            
            # 在日志中输出坐标信息
            self.get_logger().info(f'目标中心坐标: x={center_x:.2f}, y={center_y:.2f}')
            
            # 在图像上标记中心点
            cv2.circle(annotated_frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
        else:
            # 如果没有检测到目标，则发布默认坐标
            self.target_publisher.publish(self.point_msg)
        
        # 将处理后的图像转换为ROS消息并发布
        detected_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        detected_msg.header.stamp = self.get_clock().now().to_msg()
        detected_msg.header.frame_id = "camera_frame"
        self.image_publisher.publish(detected_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()