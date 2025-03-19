#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 创建订阅者，接收图像
        self.subscription = self.create_subscription(
            Image,
            '/custom_ns/custom_camera/custom_image',  # 根据实际的话题名修改
            self.image_callback,
            10)
        
        # 创建发布者，发布处理后的图像
        self.publisher = self.create_publisher(
            Image,
            '/yolo/detected_image',
            10)
        
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp4/weights/best.pt')
        
        self.get_logger().info('YOLO detector node has been started')

    def image_callback(self, msg):
        # 将ROS图像消息转换为OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 运行YOLO检测
        results = self.model(cv_image)
        
        # 在图像上绘制检测结果
        annotated_frame = results[0].plot()
        
        # 将处理后的图像转换回ROS消息并发布
        detected_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        self.publisher.publish(detected_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetector()
    rclpy.spin(yolo_detector)
    yolo_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()