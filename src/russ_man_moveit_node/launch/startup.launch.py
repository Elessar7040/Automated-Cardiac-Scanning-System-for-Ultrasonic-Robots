from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='russ_man_moveit_node').find('russ_man_moveit_node')
    
    # 获取launch文件路径
    gazebo_launch_path = os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
    moveit_launch_path = os.path.join(pkg_share, 'launch', 'my_moveit_rviz.launch.py')

    # 包含其他launch文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path)
    )

    obstacle_service_node = Node(
        package='obstacle_node',
        executable='obstacle_service',
        name='obstacle_service_node',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )
    cartesian_abs_action_server_node = Node(
        package='planning_node',
        executable='cartesian_abs_action_server',
        name='cartesian_abs_action_server_node',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )
    cartesian_abs_action_client_node = Node(
        package='planning_node',
        executable='cartesian_abs_action_client',
        name='cartesian_abs_action_client_node',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )
    cartesian_rel_action_server_node = Node(
        package='end_control_node',
        executable='cartesian_rel_action_server',
        name='cartesian_rel_action_server_node',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )
    waypoint_action_server_node = Node(
        package='planning_node',
        executable='waypoint_action_server',
        name='waypoint_action_server_node',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )
    end_control_node = Node(
        package='end_control_node',
        executable='end_control_service',
        name='end_control_service',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )
    cartesian_linear_action_server_node = Node(
        package='planning_node',
        executable='cartesian_linear_action_server',
        name='cartesian_linear_action_server_node',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )
    cartesian_linear_action_client_test_node = Node(
        package='planning_node',
        executable='cartesian_linear_action_client_test',
        name='cartesian_linear_action_client_test_node',
        parameters=[
            {"use_sim_time": True}
        ],
        output='screen'
    )

    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加动作
    ld.add_action(gazebo_launch)
    ld.add_action(moveit_launch)

    # 可同时开启，实现控制接口 abs, rel, 实现障碍物避障与路径规划配置
    # ld.add_action(obstacle_service_node)
    ld.add_action(cartesian_abs_action_server_node)
    # ld.add_action(cartesian_abs_action_client_node)
    ld.add_action(cartesian_rel_action_server_node)
    # ld.add_action(cartesian_linear_action_server_node)
    # ld.add_action(cartesian_linear_action_client_test_node)
    # 开启后，可实现控制接口 waypoint
    ld.add_action(waypoint_action_server_node)
    ld.add_action(end_control_node)

    return ld 