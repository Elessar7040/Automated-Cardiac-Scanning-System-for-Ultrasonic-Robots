'''
Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
Date: 2025-01-02 11:57:29
LastEditors: “feiyang_hong” “feiyang.hong@infinityrobot.cn”
LastEditTime: 2025-01-08 17:33:32
FilePath: /planning_control_node/src/moveit_node/launch/demo_test.launch.py
Description: 
'''

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino5_v6_robot", package_name="moveit_node").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)

    moveit_cartesian_node = Node(
        package='planning_node',
        executable='moveit_cartesian_test_2',
        name='moveit_cartesian_node',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    moveit_obstacle_avoid_node = Node(
        package='planning_node',
        executable='moveit_obstacle_avoid_test',
        name='moveit_obstacle_avoid_node',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    obstacle_test_1_node = Node(
        package='obstacle_node',
        executable='obstacle_test_1',
        name='obstacle_test_1',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    obstacle_service_node = Node(
        package='obstacle_node',
        executable='obstacle_service',
        name='obstacle_service_node',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    cartesian_abs_action_server_node = Node(
        package='planning_node',
        executable='cartesian_abs_action_server',
        name='cartesian_abs_action_server_node',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    cartesian_abs_action_client_node = Node(
        package='planning_node',
        executable='cartesian_abs_action_client',
        name='cartesian_abs_action_client_node',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    cartesian_rel_action_server_node = Node(
        package='planning_node',
        executable='cartesian_rel_action_server',
        name='cartesian_rel_action_server_node',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    waypoint_action_server_node = Node(
        package='planning_node',
        executable='waypoint_action_server',
        name='waypoint_action_server_node',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )
    end_control_node = Node(
        package='end_control_node',
        executable='end_control_service',
        name='end_control_service',
        parameters=[
            {"use_sim_time": False}
        ],
        output='screen'
    )

    # ld.add_action(moveit_cartesian_node)
    # ld.add_action(moveit_obstacle_avoid_node)
    
    # 可同时开启，实现控制接口 abs, rel, 实现障碍物避障与路径规划配置
    # ld.add_action(obstacle_test_1_node)

    ld.add_action(obstacle_service_node)
    ld.add_action(cartesian_abs_action_server_node)
    ld.add_action(cartesian_abs_action_client_node)
    ld.add_action(cartesian_rel_action_server_node)
    
    # 开启后，可实现控制接口 waypoint
    ld.add_action(waypoint_action_server_node)
    ld.add_action(end_control_node)
    
    return ld
