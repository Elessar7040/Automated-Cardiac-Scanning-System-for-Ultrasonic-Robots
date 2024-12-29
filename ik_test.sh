ros2 action send_goal /ur_group_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [
      { positions: [-7.912265921429266e-05, -0.5877330642601567, 0.6423155910008045, 0.10022157273127025, -4.092580262513701e-06, -0.15456798750666928], time_from_start: { sec: 0, nanosec: 100000000 } },
    ]
  }
}"

