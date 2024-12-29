import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from tf2_ros import TransformListener, Buffer
# from launch.actions import ExecuteProcess
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class InverseKinematicsClient(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_client')
        self.tf_buffer = Buffer()
        self.tf_trans = {}
        self.tf_rotation = {}
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.match_point = [0.2, 0.3, 0.4]

        self.timer = self.create_timer(0.01, self.tf_callback)
        self.joint_state = JointState()
        self.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 机械臂初始位置
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        self.subscription_match_point = self.create_subscription(Point, '/match_point', self.match_point_callback, 10)
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
    
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
        self.match_point[0] = msg.x
        self.match_point[1] = msg.y
        self.match_point[2] = msg.z
        self.get_logger().info(f"Received coordinates: ({msg.x}, {msg.y}, {msg.z})")
        self.compute_ik(self.match_point)
    
    def compute_ik(self, target_position):
        # 构建服务请求
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur_group'  # 替换为你的规划组
        request.ik_request.pose_stamped.header.frame_id = 'base_link'  # 替换为基坐标系
        request.ik_request.pose_stamped.pose.position.x = target_position[0]
        request.ik_request.pose_stamped.pose.position.y = 0.19139
        request.ik_request.pose_stamped.pose.position.z = target_position[2]
        
        request.ik_request.robot_state.joint_state = self.joint_state
        # xyzw: 8.3482e-05; 8.3475e-05; 0.70708; 0.70713
        # 2.7361e-06; 2.7366e-06; 0.70711; 0.70711
        # request.ik_request.pose_stamped.pose.orientation.x = self.tf_rotation["x"]
        # request.ik_request.pose_stamped.pose.orientation.y = self.tf_rotation["y"]
        # request.ik_request.pose_stamped.pose.orientation.z = self.tf_rotation["z"]
        # request.ik_request.pose_stamped.pose.orientation.w = self.tf_rotation["w"]
        request.ik_request.pose_stamped.pose.orientation.x = 2.7361e-06
        request.ik_request.pose_stamped.pose.orientation.y = 2.7366e-06
        request.ik_request.pose_stamped.pose.orientation.z = 0.70711
        request.ik_request.pose_stamped.pose.orientation.w = 0.70711
        # request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # request.ik_request.pose_stamped.pose.orientation.y = 0.0
        # request.ik_request.pose_stamped.pose.orientation.z = 0.0
        # request.ik_request.pose_stamped.pose.orientation.w = 1.0

        # 发送请求并等待结果
        # future = self.cli.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        # self.get_logger().info(f'Waiting for client')
        future = self.cli.call_async(request)
        # self.get_logger().info(f'Waiting for response')
        future.add_done_callback(self.handle_ik_response)
        # if future.result():
        #     self.handle_ik_response(future)
        # rclpy.spin(self, future)

        # if future.result():
        #     joint_positions = future.result().solution.joint_state.position
        #     # [0.13202251126304942, -0.9597681527318485, 1.0878146169188283, -0.12804647355941298, -1.7028188380559341, -7.166965476031702e-08]
        #     # [-7.912265921429266e-05, -0.5877330642601567, 0.6423155910008045, 0.10022157273127025, -4.092580262513701e-06, -0.15456798750666928]
        #     self.get_logger().info(f"Joint positions: {joint_positions}")
        #     # start_gazebo_cmd =  ExecuteProcess(
        #     #     cmd=['ros2', 'action', 'send_goal', '/ur_group_controller/follow_joint_trajectory', 'control_msgs/action/FollowJointTrajectory',
        #     #     '{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],points: [{ positions: [-7.912265921429266e-05, -0.5877330642601567, 0.6423155910008045, 0.10022157273127025, -4.092580262513701e-06, -0.15456798750666928], time_from_start: { sec: 0, nanosec: 100000000 } },]}}"'],
        #     #     output='screen')
        #     return joint_positions
        # else:
        #     self.get_logger().error("Failed to compute IK")
        #     return None
        
#     ros2 action send_goal /ur_group_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
#         trajectory: {
#         joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
#         points: [
#       { positions: [-7.912265921429266e-05, -0.5877330642601567, 0.6423155910008045, 0.10022157273127025, -4.092580262513701e-06, -0.15456798750666928], time_from_start: { sec: 0, nanosec: 100000000 } },
#     ]
#   }
# }"

    def handle_ik_response(self, future):
        self.get_logger().info(f'Entering handle_ik_response')
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                joint_state = JointState()
                # joint_state.name = response.solution.joint_state.name
                joint_state.position = response.solution.joint_state.position

                self.send_trajectory(joint_state.position)

                # self.publisher.publish(joint_state)
                self.get_logger().info(f'Joint states published: {joint_state.position}')
            else:
                self.get_logger().error('Failed to compute IK solution.')
        except Exception as e:
            self.get_logger().error(f'Error while calling IK service: {e}')


    def send_trajectory(self, joint_positions):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 0  # 时间为即时执行
        point.time_from_start.nanosec = 3000000000  # 3000ms
        trajectory_msg.points.append(point)
        self.trajectory_pub.publish(trajectory_msg)
        

def main(args=None):
    rclpy.init(args=args)
    ik_client = InverseKinematicsClient()
    # 0.75996; 0.19139; 0.39693
    # target_point = [0.82, 0.19139, 0.40]
    # target_point = [0.74996, 0.19139, 0.39753]
    rclpy.spin(ik_client)
    # ik_client.compute_ik(target_point)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
