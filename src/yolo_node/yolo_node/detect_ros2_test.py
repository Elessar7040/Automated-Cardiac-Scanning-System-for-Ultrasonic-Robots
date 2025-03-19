#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_test')
        
        # 创建发布者，发布处理后的图像
        self.publisher = self.create_publisher(
            Image,
            '/yolo/detected_image',
            1)
        
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp4/weights/best.pt')
        
        # 打开视频文件
        self.video_path = '/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/Cardiac_15.avi'
        if not os.path.exists(self.video_path):
            self.get_logger().error(f'视频文件不存在: {self.video_path}')
            return
            
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error('无法打开视频文件')
            return
            
        # 创建定时器，定期处理视频帧
        self.timer = self.create_timer(0.033, self.process_frame)  # 约30fps
        
        self.get_logger().info('YOLO检测器节点已启动，正在处理视频文件')

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