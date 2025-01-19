import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from tf2_ros import TransformListener, Buffer
import transforms3d as tfs
from moveit_msgs.srv import GetPositionIK

import numpy as np
import open3d as o3d
import copy
import time


class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__("point_cloud_processor")
        self.joint_position = {}

        self.tf_buffer = Buffer()
        self.tf_trans = {}
        self.tf_rotation = {}
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.tf_callback)

        self.match_point = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.normal_vector = []


        self.joint_name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.o3d_class = o3d.geometry.PointCloud()
        
        self.subscription_point_cloud = self.create_subscription(PointCloud2, "/custom_ns/custom_camera/custom_points", self.callback, 1)
        self.subscription_match_point = self.create_subscription(Point, '/match_point', self.match_point_callback, 5)
        # self.subscription_joint_state = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        # self.publisher_point_cloud = self.create_publisher(PointCloud2, "/transformed_point_cloud", 10)
        self.publisher_point_cloud = self.create_publisher(PointCloud2, "/transformed_point_cloud", 1)
        self.publisher_downsampled_point_cloud = self.create_publisher(PointCloud2, "/transformed_downsampled_point_cloud", 1)
        # self.match_point_publisher = self.create_publisher(Point, '/match_point_xyz', 10)
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window()

    def callback(self, msg):
        try:
            points = []
            # self.get_logger().info(f"msg.frame_id: {msg.header.frame_id}")
            # self.get_logger().info(f"Time_before_points: {self.get_clock().now()}")
            for point in point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
                points.append([point[0], point[1], point[2]])
            # self.get_logger().info(f"msg: {msg}")
            # self.get_logger().info(f"Time_after_points: {self.get_clock().now()}")
            points_np = np.array(points)
            # self.get_logger().info(f"Received point cloud with {points_np.shape[0]} points")

            if points_np.shape[0] > 0:
                # self.get_logger().info(f"Time_before_mean: {self.get_clock().now()}")
                center = np.mean(points_np, axis=0)
                # self.get_logger().info(f"Time_after_mean: {self.get_clock().now()}")
                # self.get_logger().info(f"Point Cloud Center: {center}")
            self.process_with_open3d(points_np)
        except Exception as e:
            self.get_logger().error(f"Failed to process point cloud: {e}")
    '''
    [point_cloud_processor]: msg: sensor_msgs.msg.JointState(
        header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=221, nanosec=937000000), frame_id=''), 
        name=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], 
        position=[-5.3924776350910975e-05, 0.03289854317428276, -0.03672762077583602, 0.0038456593169877706, 5.293661777017178e-05, -1.801192104888827e-05], 
        velocity=[0.00017341246369371885, 0.026558936262500905, -0.029276949313537085, 0.003292128657039561, -0.00015153184098510759, -3.326324025232342e-05], 
        effort=[nan, nan, nan, nan, nan, nan]
    )
    '''

    # def joint_state_callback(self, msg):
    #     # self.get_logger().info(f"msg: {msg}")
    #     for joint_name, joint_position in zip(msg.name, msg.position):
    #         # self.get_logger().info(f"Joint: {joint_name}, Position: {joint_position}")
    #         self.joint_position[joint_name] = joint_position
    #     # self.get_logger().info(f"Position: {self.joint_position}")
    #     T = np.eye(4) 
    #     T[ :3, :3] = self.o3d_class.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0)) # 旋转矩阵

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


    def match_point_callback(self, msg):
        self.match_point['x'] = msg.x
        self.match_point['y'] = msg.y
        self.match_point['z'] = msg.z
        # self.get_logger().info(f"Received coordinates: ({msg.x}, {msg.y}, {msg.z})")



    def process_with_open3d(self, points_np):
        # # 将 numpy 数组转换为 Open3D 点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_np)

        # # # 简单的可视化
        # o3d.visualization.draw_geometries([pcd])

        # self.get_logger().info(f"Time_before_downsample: {self.get_clock().now()}")

        # # # 示例：点云滤波
        downsampled_pcd_T = pcd.voxel_down_sample(voxel_size=0.05)

        # self.get_logger().info(f"Time_after_downsample: {self.get_clock().now()}")

        # o3d.visualization.draw_geometries([downsampled_pcd])
        if len(downsampled_pcd_T.points) == 0:
            self.get_logger().warn("Downsampled point cloud is empty!")

        # downsampled_pcd_T = copy.deepcopy(downsampled_pcd)
        # T_base2ee = np.eye(4) 
        # T_base2ee[ :3, :3] = tfs.quaternions.quat2mat([self.tf_rotation["w"],self.tf_rotation["x"],self.tf_rotation["y"],self.tf_rotation["z"]])
        # T1[ :3, :3] = pcd.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0)) # 旋转矩阵
        # T_ee2pc = np.eye(4)
        # T_ee2pc[ :3, :3] = pcd.get_rotation_matrix_from_xyz((-np.pi / 2, 0, -np.pi / 2)) # 旋转矩阵
        
        # -0.81694; -0.19139; -0.16929
        # T_base2ee[0,3] = self.tf_trans["x"] # 平移向量的dx 
        # T_base2ee[1,3] = self.tf_trans["y"] # 平移向量的dy
        # T_base2ee[2,3] = self.tf_trans["z"] # 平移向量的dz

        # T_base2pc = np.dot(np.dot(T_base2ee, T_ee2pc), T_base2ee.T)
        # T_base2pc = np.dot(T_base2ee, T_ee2pc)

        T = np.eye(4)
        T[:3, :3] = pcd.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0)) # 旋转矩阵
        T[0,3] = self.tf_trans["x"] # 平移向量的dx
        T[1,3] = self.tf_trans["y"] # 平移向量的dy
        T[2,3] = self.tf_trans["z"] # 平移向量的dz

        T_relative2base = np.eye(4)
        T_relative2base[0,3] = -self.tf_trans["x"] # 平移向量的dx 
        T_relative2base[1,3] = -self.tf_trans["y"] # 平移向量的dy
        T_relative2base[2,3] = -self.tf_trans["z"] # 平移向量的dz

        # T_result_1 = np.dot(T, T_relative2base)
        T_result_1 = np.dot(T_relative2base, T)

        T_rotate = np.eye(4) 
        T_rotate[ :3, :3] = tfs.quaternions.quat2mat([self.tf_rotation["w"],self.tf_rotation["x"],self.tf_rotation["y"],self.tf_rotation["z"]])

        # T_result_2 = np.dot(T_result_1, T_rotate)
        T_result_2 = np.dot(T_rotate, T_result_1)

        T_rotate_2 = np.eye(4) 
        T_rotate_2[ :3, :3] = pcd.get_rotation_matrix_from_xyz((0, 0, -np.pi / 2))

        # T_result_3 = np.dot(T_result_2, T_rotate_2)
        T_result_3 = np.dot(T_rotate_2, T_result_2)

        T_relative2base_2 = np.eye(4)
        T_relative2base_2[0,3] = self.tf_trans["x"] # 平移向量的dx 
        T_relative2base_2[1,3] = self.tf_trans["y"] # 平移向量的dy
        T_relative2base_2[2,3] = self.tf_trans["z"] # 平移向量的dz

        # T_result = np.dot(T_result_3, T_relative2base_2)
        T_result = np.dot(T_relative2base_2, T_result_3)



        # downsampled_pcd_T.transform(T)
        # downsampled_pcd_T.transform(T_relative2base)
        # downsampled_pcd_T.transform(T_rotate)
        # downsampled_pcd_T.transform(T_rotate_2)
        # downsampled_pcd_T.transform(T_relative2base_2)
        # self.get_logger().info(f"Time_before_transform: {self.get_clock().now()}")
        downsampled_pcd_T.transform(T_result)
        # self.get_logger().info(f"Time_after_transform: {self.get_clock().now()}")

        radius = 0.5 # 搜索半径 
        max_nn = 10 # 邻域内用于估算法线的最大点数 
        # print("estimating_normals")
        downsampled_pcd_T.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn)) # 执行法线估计 
        # # o3d.visualization.draw_geometries([downsampled_pcd_T], point_show_normal=True) 

        # print("estimate_normals OK")

        downsampled_pcd_T_tree = o3d.geometry.KDTreeFlann(downsampled_pcd_T)
        # print("KDTreeFlann OK")

        target_point = np.array([self.match_point['x'], self.match_point['y'], self.match_point['z']])
        # print("target_point OK", target_point)

        # 查询最近的点索引
        [_, idx, _] = downsampled_pcd_T_tree.search_knn_vector_3d(target_point, 1)
        # print("search_knn_vector_3d OK")
        nearest_idx = idx[0]

        # Step 3: 获取最近点的法向量
        normal = downsampled_pcd_T.normals[nearest_idx]
        # print("search_knn_vector_3d OK")

        self.normal_vector = normal
        print("normal_vector: ", self.normal_vector)

        # downsampled_pcd_T.transform(T_base2ee)
        # downsampled_pcd_T.transform(T_ee2pc)
        # downsampled_pcd_T.transform(T_base2pc)
        # self.get_logger().info(f"Time_before_downsample_array: {self.get_clock().now()}")
        transformed_downsampled_points = np.asarray(downsampled_pcd_T.points)
        # self.get_logger().info(f"Time_after_downsample_array: {self.get_clock().now()}")
        # self.get_logger().info(f"Transformed point cloud size: {len(downsampled_pcd_T.points)}")

        # # self.publish_transformed_cloud(np.asarray(downsampled_pcd_T.points))

        # pcd.transform(T)
        # pcd.transform(T_relative2base)
        # pcd.transform(T_rotate)
        pcd.transform(T_result)
        # self.get_logger().info(f"Time_before_array: {self.get_clock().now()}")
        transformed_points = np.asarray(pcd.points)


        # radius = 0.01 # 搜索半径 
        # max_nn = 30 # 邻域内用于估算法线的最大点数 
        # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn)) # 执行法线估计 
        # o3d.visualization.draw_geometries([pcd], point_show_normal=True)
        
        # self.get_logger().info(f"Time_after_array: {self.get_clock().now()}")
        # self.get_logger().info(f"Transformed point cloud size: {len(pcd.points)}")

        self.publish_transformed_cloud(transformed_points, transformed_downsampled_points)

        # Clear previous geometries (if any)
        # self.vis.clear_geometries()
        

        # Add the point cloud to the visualizer
        # self.vis.add_geometry(pcd)

        # self.vis.run()

        # Keep updating the visualizer with new data
        # while True:
        # self.vis.update_geometry(pcd)  # Update the point cloud geometry
        # self.vis.poll_events()         # Poll for window events
        # self.vis.update_renderer()     # Refresh the renderer

        # o3d.io.write_point_cloud("src/opencv_description/pointcloud_output/output.ply", downsampled_pcd_T)
        # o3d.io.write_point_cloud("src/opencv_description/pointcloud_output/output.pcd", downsampled_pcd_T)
        # o3d.io.write_point_cloud("src/opencv_description/pointcloud_output/output.xyz", downsampled_pcd_T)
            
            # Optionally, exit after a certain time or condition
        # time.sleep(1)  # Delay to control refresh rate

            # You can add a stopping condition here if needed
            # For example, to stop after 10 seconds:
            # if time.time() - start_time > 10:
            #     break
        # vis.destroy_window()  # Close the visualization window after the loop

        # pcd = o3d.io.read_point_cloud("src/opencv_description/pointcloud_output/output.pcd")
        # transformed_points = np.asarray(pcd.points)
        # self.publish_transformed_cloud(transformed_points)

    def publish_transformed_cloud(self, transformed_points, transformed_downsampled_points):
        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        cloud_msg = point_cloud2.create_cloud_xyz32(header, transformed_points)
        downsampled_cloud_msg = point_cloud2.create_cloud_xyz32(header, transformed_downsampled_points)
        self.publisher_point_cloud.publish(cloud_msg)
        self.publisher_downsampled_point_cloud.publish(downsampled_cloud_msg)


def main(args=None):
    rclpy.init()
    node = PointCloudProcessor()
    rclpy.spin(node)
    rclpy.shutdown()