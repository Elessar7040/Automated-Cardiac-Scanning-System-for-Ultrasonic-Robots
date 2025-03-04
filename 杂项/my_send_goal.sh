ros2 action send_goal /ur_group_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [
      { positions: [0, 0, 0, 0, 0, 0], time_from_start: { sec: 0, nanosec: 100000000 } },
      { positions: [0, -1.57, 0, 0, 0, 0], time_from_start: { sec: 1, nanosec: 1000000000 } },
      { positions: [0, -1.57, 0.785, 0, 0, 0], time_from_start: { sec: 2, nanosec: 1000000000 } },
      { positions: [0, -1.57, 0.785, 0.785, 0, 0], time_from_start: { sec: 3, nanosec: 1000000000 } },
    ]
  }
}"

