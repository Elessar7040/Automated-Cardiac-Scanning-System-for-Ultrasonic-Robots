import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped


class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /compute_ik not available, waiting...')

    def compute_ik(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_group"  # 替换为你的规划组名称
        request.ik_request.robot_state.joint_state.name = []  # 可以填入当前关节状态
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout.sec = 2  # 设置超时时间

        self.get_logger().info("Sending IK request...")
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(f"IK Solution: {future.result().solution.joint_state.position}")
        else:
            self.get_logger().error("Failed to compute IK solution.")


def main():
    rclpy.init()
    node = IKClient()
    # 0.69529; 0.19151; 0.48182
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"  # 替换为基坐标系
    target_pose.pose.position.x = 0.69529
    target_pose.pose.position.y = 0.19151
    target_pose.pose.position.z = 0.48182
    target_pose.pose.orientation.w = 1.0

    node.compute_ik(target_pose)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
