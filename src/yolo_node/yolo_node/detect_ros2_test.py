#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_test')

        # 创建并发布Point消息
        self.point_msg = Point()
        self.point_msg.x = 320.0
        self.point_msg.y = 224.0
        self.point_msg.z = 0.0  # 2D图像中z坐标设为0
        
        # 创建发布者，发布处理后的图像
        self.publisher = self.create_publisher(
            Image,
            '/yolo/detected_image',
            1)
        
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp4/weights/best.pt')
        
        # 打开视频文件
        # self.video_path = '/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/Cardiac_15.avi'
        self.video_path = '/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/Cardiac_14.avi'
        if not os.path.exists(self.video_path):
            self.get_logger().error(f'视频文件不存在: {self.video_path}')
            return
            
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error('无法打开视频文件')
            return
        
        # 创建发布者，发布目标中心坐标
        self.target_publisher = self.create_publisher(
            Point,
            '/yolo/target_center',
            10)
        
        self.target_subscriber = self.create_subscription(
            Image,
            '/yolo/detected_image',
            self.detected_image_callback,
            10)
            
        # 创建定时器，定期处理视频帧
        self.timer = self.create_timer(0.033, self.process_frame)  # 约30fps
        
        self.get_logger().info('YOLO检测器节点已启动，正在处理视频文件')

    def detected_image_callback(self, msg):
        self.image_width_ = msg.width
        self.image_height_ = msg.height
        self.get_logger().info(f'图像尺寸: {self.image_width_}x{self.image_height_}')

    def process_frame(self):
        # 读取视频帧
        ret, frame = self.cap.read()
        
        if not ret:
            # 视频结束，重新开始
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.get_logger().info('视频已播放完毕，重新开始')
            return
        
        # 运行YOLO检测
        results = self.model(frame)
        
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
            self.point_msg.x = float(center_x)
            self.point_msg.y = float(center_y)
            self.point_msg.z = 0.0  # 2D图像中z坐标设为0
            
            self.target_publisher.publish(self.point_msg)
            
            # 在日志中输出坐标信息
            self.get_logger().info(f'目标中心坐标: x={center_x:.2f}, y={center_y:.2f}')
            
            # 在图像上标记中心点
            cv2.circle(annotated_frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
        else:
            self.get_logger().info('未检测到目标')
            self.target_publisher.publish(self.point_msg)
        
        # 将处理后的图像转换为ROS消息并发布
        detected_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        detected_msg.header.stamp = self.get_clock().now().to_msg()
        detected_msg.header.frame_id = "camera_frame"
        self.publisher.publish(detected_msg)

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()