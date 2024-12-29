import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
from moveit_msgs.srv import GetPositionIK
import transforms3d as tfs

# Tool for transferring to ROS2 message type Image
bridge = CvBridge()

class opencv_node(Node):
    def __init__(self, name):
        super().__init__(name)
        self.tf_buffer = Buffer()
        self.tf_trans = {}
        self.tf_rotation = {}
        self.depth_img = None
        self.P_base = [0.4, 0.3, 0.2]
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.tf_callback)
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
        self.subscription_color_image = self.create_subscription(Image, "/custom_ns/custom_camera/custom_image", self.callback, 10)
        self.subscription_depth_image = self.create_subscription(Image, "/custom_ns/custom_camera/depth/image_raw", self.callback2, 10)
        self.match_point_publisher = self.create_publisher(Point, '/match_point', 10)
        self.timer = self.create_timer(1.0, self.publish_match_point)  # 每秒发布一次
        # self.publisher_match_points = self.create_publisher(int, "/transformed_point_cloud", 1)
        self.get_logger().info(f"{name} Started")

    def callback(self, data):
        try:
            cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
            # cv2.imshow("color_image", cv_img)
            # cv2.waitKey(1)
            # image_a_path = "src/opencv_description/opencv_description/img1.jpg"
            image_match_path = "src/opencv_description/match_pictures/Painter2_match.png"
            scale_factors = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5]  # 尝试的缩放比例

            # 执行代码
            # result_img, best_score = self.find_most_similar_region(image_a_path, image_match_path, scale_factors)
            result_img, best_score = self.find_most_similar_region(cv_img, image_match_path, scale_factors)
            cv2.imshow("Result", result_img)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process color_image: {e}")
    
    def callback2(self, data):
        try:
            self.depth_img = bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
            # cv2.imshow("depth_image", self.depth_img)
            # x, y = 320, 240
            # depth_value = depth_img[y, x]
            # self.get_logger().info(f"Depth at pixel ({x}, {y}): {depth_value} meters")
            normalized_depth = cv2.normalize(self.depth_img, None, 0, 255, cv2.NORM_MINMAX)

            # 像素坐标 (u, v) 和深度值 Z
            u, v = 135, 30
            # Z = depth_img[v, u]  # 深度值
            Z = self.depth_img[v, u]  # 深度值

            # self.tf_base2image_vuz(u, v, Z)

            normalized_depth = normalized_depth.astype(np.uint8)
            # cv2.imshow("normalized_depth_image", normalized_depth)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth_image: {e}")
    
    def tf_callback(self):
        try:
            # 查询基坐标系和末端坐标系之间的变换
            from_frame = 'base_link'
            to_frame = 'kinect_link'
            now = rclpy.time.Time()
            
            # 从 tf_buffer 获取变换
            transform = self.tf_buffer.lookup_transform(from_frame, to_frame, now)
            
            # 解析位置和旋转
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            self.tf_trans["x"] = translation.x
            self.tf_trans["y"] = translation.y
            self.tf_trans["z"] = translation.z
            self.tf_rotation["x"] = rotation.x
            self.tf_rotation["y"] = rotation.y
            self.tf_rotation["z"] = rotation.z
            self.tf_rotation["w"] = rotation.w

            # self.get_logger().info(f"Translation: x={translation.x}, y={translation.y}, z={translation.z}")
            # self.get_logger().info(f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")
        
        except Exception as e:
            self.get_logger().error(f"Could not transform {from_frame} to {to_frame}: {e}")

    def tf_base2image_vuz(self, P_image_v, P_image_u, P_image_z):
        # 焦距在x方向上的缩放因子
        fx = 548.81623
        # 焦距在x方向上的缩放因子
        fy = 548.65548
        # 主点在x方向上的坐标
        # cx = 316.76451
        cx = 320
        # 主点在y方向上的坐标
        # cy = 240.48396
        cy = 240

        # 像素坐标 (u, v) 和深度值 Z
        u, v = P_image_u, P_image_v
        # Z = depth_img[v, u]  # 深度值
        Z = P_image_z  # 深度值

        # 转换为相机坐标
        X_cam = Z * (u - cx) / fx
        Y_cam = Z * (v - cy) / fy
        Z_cam = Z

        P_image = np.asarray([X_cam, Y_cam, Z_cam, 1]).reshape(4, 1)

        T_base2ee = np.eye(4)
        T_base2ee[ :3, :3] = tfs.quaternions.quat2mat([self.tf_rotation["w"],self.tf_rotation["x"],self.tf_rotation["y"],self.tf_rotation["z"]])
        T_base2ee[0,3] = self.tf_trans["x"] # 平移向量的dx 
        T_base2ee[1,3] = self.tf_trans["y"] # 平移向量的dy
        T_base2ee[2,3] = self.tf_trans["z"] # 平移向量的dz

        # T_ee2image = np.asarray([0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1]).reshape(4, 4)
        T_ee2image = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])  # 通过列表直接创建矩阵

        # 0.81692; 0.19139; 0.1687
        T_base2image = np.dot(T_base2ee, T_ee2image)
        # print(f"Camera coordinates: ({Y_cam_1}, {Y_cam_2}, {Y_cam_3})")
        P_base = np.dot(T_base2image, P_image)
        self.P_base = [P_base[0][0], P_base[1][0], P_base[2][0]]
        print(f"Camera coordinates: ({self.P_base[0]}, {self.P_base[1]}, {self.P_base[2]})")
        # print(f"Camera coordinates: ({P_base[0][0]}, {P_base[1][0]}, {P_base[2][0]})")


    
    def find_most_similar_region(self, image_origin, image_match, scale_factors):
        # 读取图片
        # img_a = cv2.imread(image_origin)
        img_a = image_origin
        img_match = cv2.imread(image_match)

        # 转为灰度图
        gray_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
        gray_b = cv2.cvtColor(img_match, cv2.COLOR_BGR2GRAY)

        best_match = None
        best_score = -100000000
        best_rect = None

        # 遍历不同缩放比例
        for scale in scale_factors:
            scaled_b = cv2.resize(gray_b, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            if scaled_b.shape[0] >= gray_a.shape[0] or scaled_b.shape[1] >= gray_a.shape[1]:
                break
            res = cv2.matchTemplate(gray_a, scaled_b, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            if max_val > best_score:
                best_score = max_val
                best_match = scaled_b
                best_rect = (max_loc[0], max_loc[1], scaled_b.shape[1], scaled_b.shape[0])

        # 在图片A上绘制矩形
        top_left = (best_rect[0], best_rect[1])
        bottom_right = (best_rect[0] + best_rect[2], best_rect[1] + best_rect[3])
        u_center = int((top_left[0] + bottom_right[0]) / 2)
        v_center = int((top_left[1] + bottom_right[1]) / 2)

        self.tf_base2image_vuz(v_center, u_center, self.depth_img[v_center, u_center])
        
        result_img = img_a.copy()
        cv2.rectangle(result_img, top_left, bottom_right, (0, 255, 0), 2)
        # cv2.rectangle(result_img, (135, 30), (495, 300), (0, 255, 0), 2)
        # print(f"top_left, bottom_right: ({top_left}, {bottom_right})")

        return result_img, best_score
    
        
    def publish_match_point(self):
        msg = Point()
        msg.x = self.P_base[0]  # 替换为实际的 x 坐标
        msg.y = self.P_base[1]  # 替换为实际的 y 坐标
        msg.z = self.P_base[2]  # 替换为实际的 z 坐标
        self.match_point_publisher.publish(msg)
        self.get_logger().info(f"Published coordinates: ({msg.x}, {msg.y}, {msg.z})")
    

def main(args=None):
    rclpy.init()
    node = opencv_node("opencv_node")
    rclpy.spin(node)
    rclpy.shutdown()
