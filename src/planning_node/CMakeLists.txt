cmake_minimum_required(VERSION 3.8)
project(planning_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(end_control_node REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# 添加 Python 头文件路径
include_directories(${Python3_INCLUDE_DIRS})

# 添加头文件目录
include_directories(
  include
)

# 确保头文件被安装
install(
  DIRECTORY include/
  DESTINATION include
)

# 添加action依赖
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveEndToAbsPos.action"
  "action/WaypointMotion.action"
  "srv/GlobPlanningSetting.srv"
  "msg/Waypoint.msg"
  DEPENDENCIES geometry_msgs
)
# 生成导出依赖
rosidl_get_typesupport_target(cpp_typesupport_target 
  "${PROJECT_NAME}" "rosidl_typesupport_cpp"
)

add_executable(moveit_cartesian_test 
  test/moveit_cartesian_test.cpp
)
add_executable(moveit_cartesian_test_2 
  test/moveit_cartesian_test_2.cpp
)
add_executable(pose_publisher 
  test/pose_publisher.cpp
)
add_executable(moveit_obstacle_avoid_test 
  test/moveit_obstacle_avoid_test.cpp
)
add_executable(cartesian_abs_action_server
  src/cartesian_abs_action_server.cpp
)
add_executable(cartesian_abs_action_client
  src/cartesian_abs_action_client.cpp
)
add_executable(cartesian_abs_action_client_test
  test/cartesian_abs_action_client_test.cpp
)
add_executable(waypoint_action_server
  src/waypoint_action_server.cpp
)
add_executable(waypoint_action_client_test_1
  test/waypoint_action_client_test_1.cpp
)
add_executable(waypoint_action_client_test_2
  test/waypoint_action_client_test_2.cpp
)
add_executable(match_point_converter
  src/match_point_converter.cpp
)
add_executable(ik_planning_test test/ik_planning_test.cpp)
target_include_directories(ik_planning_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ik_planning_test PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ik_planning_test
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
)
add_executable(trajectory_smooth_test test/trajectory_smooth_test.cpp)
ament_target_dependencies(trajectory_smooth_test
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
)
add_executable(waypoint_action_client_test_3
  test/waypoint_action_client_test_3.cpp
)
ament_target_dependencies(waypoint_action_client_test_3
  rclcpp
  rclcpp_action
)

ament_target_dependencies(moveit_cartesian_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)
ament_target_dependencies(pose_publisher
  rclcpp
  geometry_msgs
)
ament_target_dependencies(moveit_cartesian_test_2
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)
ament_target_dependencies(moveit_obstacle_avoid_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  moveit_msgs
)
ament_target_dependencies(cartesian_abs_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
)
ament_target_dependencies(cartesian_abs_action_client
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
)
ament_target_dependencies(cartesian_abs_action_client_test
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)
ament_target_dependencies(waypoint_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  end_control_node
)
ament_target_dependencies(waypoint_action_client_test_1
  rclcpp
  rclcpp_action
)
ament_target_dependencies(waypoint_action_client_test_2
  rclcpp
  rclcpp_action
)
ament_target_dependencies(match_point_converter
  rclcpp
  geometry_msgs
)

# 链接生成的接口库
target_link_libraries(cartesian_abs_action_server 
  "${cpp_typesupport_target}"
)
target_link_libraries(cartesian_abs_action_client 
  "${cpp_typesupport_target}"
)
target_link_libraries(cartesian_abs_action_client_test 
  "${cpp_typesupport_target}"
)
target_link_libraries(waypoint_action_server 
  "${cpp_typesupport_target}"
)
target_link_libraries(waypoint_action_client_test_1 
  "${cpp_typesupport_target}"
)
target_link_libraries(waypoint_action_client_test_2 
  "${cpp_typesupport_target}"
)
target_link_libraries(waypoint_action_client_test_3 
  "${cpp_typesupport_target}"
)

install(TARGETS
  moveit_cartesian_test_2
  moveit_cartesian_test
  pose_publisher
  moveit_obstacle_avoid_test
  cartesian_abs_action_server
  cartesian_abs_action_client
  cartesian_abs_action_client_test
  waypoint_action_server
  waypoint_action_client_test_1
  waypoint_action_client_test_2
  waypoint_action_client_test_3
  match_point_converter
  ik_planning_test
  trajectory_smooth_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
