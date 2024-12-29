from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rclpy
from rclpy.node import Node

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/ur_group_controller/joint_trajectory', 10)

    def send_trajectory(self, joint_positions):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        msg.points = [point]
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = JointTrajectoryPublisher()
    # 0.10820653867934724, -0.08169551748988622, -0.4203977134252791, 0.502093227822689, -1.679002886518148, -3.267400868660943e-08
    # -7.912265921429266e-05, -0.5877330642601567, 0.6423155910008045, 0.10022157273127025, -4.092580262513701e-06, -0.15456798750666928
    # -7.509687364816622e-05, -0.34390872097065317, 0.10343559575524208, 0.8161410070452594, -7.714480173689268e-06, -0.5754299054114664
    # [1.154637898184664, -0.4208453703392017, -1.7668118518357252, 2.1876575210737457, -2.725434262083444, 3.262541451721753e-07]
    target_pose = [1.154637898184664, -0.4208453703392017, -1.7668118518357252, 2.1876575210737457, -2.725434262083444, 3.262541451721753e-07]
    node.send_trajectory(target_pose)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
