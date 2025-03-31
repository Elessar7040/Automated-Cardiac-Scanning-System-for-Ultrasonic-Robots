#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from std_msgs.msg import Bool  # 添加Bool消息类型

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 创建控制订阅者
        self.control_subscription = self.create_subscription(
            Bool,
            '/yolo/start_detection',
            self.control_callback,
            1)
            
        # 初始化发布者和订阅者
        self.point_msg = Point()
        self.point_msg.x = 632.0
        self.point_msg.y = 440.0
        self.point_msg.z = 0.0
        
        self.publisher = self.create_publisher(
            Image,
            '/yolo/detected_image',
            1)
            
        self.target_publisher = self.create_publisher(
            Point,
            '/yolo/target_center',
            10)
            
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp4/weights/best.pt')
        
        # 初始化视频相关变量
        self.video_path = '/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/Cardiac_14.avi'
        self.cap = None
        self.timer = None
        
        # 添加处理状态发布者
        self.status_publisher = self.create_publisher(
            Bool,
            '/yolo/processing_status',
            1)
            
        self.get_logger().info('YOLO检测节点已启动，等待控制信号')
        
    def control_callback(self, msg):
        """控制回调函数，接收到True时启动视频处理"""
        if msg.data:
            self.start_detection()
        else:
            self.stop_detection()
            
    def start_detection(self):
        """启动视频处理"""
        if self.timer is not None:
            self.get_logger().warn('检测已在进行中')
            return
            
        try:
            if not os.path.exists(self.video_path):
                raise FileNotFoundError(f'视频文件不存在: {self.video_path}')
                
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                raise RuntimeError('无法打开视频文件')
                
            # 发布处理开始状态
            status_msg = Bool()
            status_msg.data = True
            self.status_publisher.publish(status_msg)
            
            # 创建定时器开始处理视频
            self.timer = self.create_timer(0.033, self.process_frame)
            self.get_logger().info('开始视频处理')
            
        except Exception as e:
            self.get_logger().error(f'启动检测失败: {str(e)}')
            
    def process_frame(self):
        """处理视频帧"""
        if self.cap is None or not self.cap.isOpened():
            self.stop_detection()
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('视频处理完成')
            self.stop_detection()
            return
            
        # 运行YOLO检测
        results = self.model(frame)
        annotated_frame = results[0].plot()
        
        # 处理检测结果
        if len(results[0].obb.cls) > 0:
            x1, y1, x2, y2 = results[0].obb.xyxy[0].tolist()
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            self.point_msg.x = float(center_x)
            self.point_msg.y = float(center_y)
            self.target_publisher.publish(self.point_msg)
            
            cv2.circle(annotated_frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
            self.get_logger().debug(f'目标中心坐标: x={center_x:.2f}, y={center_y:.2f}')
        else:
            self.target_publisher.publish(self.point_msg)
            
        # 发布处理后的图像
        detected_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        detected_msg.header.stamp = self.get_clock().now().to_msg()
        detected_msg.header.frame_id = "camera_frame"
        self.publisher.publish(detected_msg)
        
    def stop_detection(self):
        """停止当前视频处理"""
        if self.timer:
            self.timer.cancel()
            self.timer = None
            
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
            
        # 发布处理结束状态
        status_msg = Bool()
        status_msg.data = False
        self.status_publisher.publish(status_msg)
        
        self.get_logger().info('停止视频处理')

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 