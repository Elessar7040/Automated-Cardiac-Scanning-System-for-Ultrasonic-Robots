cmake_minimum_required(VERSION 3.8)
project(end_control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(obstacle_node REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# 添加action依赖
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveEndToRelPos.action"
  "srv/EndControl.srv"
  DEPENDENCIES geometry_msgs
)
# 生成导出依赖
rosidl_get_typesupport_target(cpp_typesupport_target 
  "${PROJECT_NAME}" "rosidl_typesupport_cpp"
)

add_executable(end_control_service
  src/end_control_service.cpp
)
ament_target_dependencies(end_control_service
  rclcpp
  obstacle_node
  geometry_msgs
  sensor_msgs
  rclcpp_action
  moveit_ros_planning_interface
)
add_executable(cartesian_rel_action_server
  src/cartesian_rel_action_server.cpp
)
ament_target_dependencies(cartesian_rel_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)
add_executable(cartesian_rel_action_client
  src/cartesian_rel_action_client.cpp
)
ament_target_dependencies(cartesian_rel_action_client
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)
add_executable(cartesian_rel_action_client_test
  test/cartesian_rel_action_client_test.cpp
)
ament_target_dependencies(cartesian_rel_action_client_test
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

# 链接生成的接口
target_link_libraries(end_control_service
  "${cpp_typesupport_target}"
)
target_link_libraries(cartesian_rel_action_server 
  "${cpp_typesupport_target}"
)
target_link_libraries(cartesian_rel_action_client 
  "${cpp_typesupport_target}"
)
target_link_libraries(cartesian_rel_action_client_test 
  "${cpp_typesupport_target}"
)

install(TARGETS
  end_control_service
  cartesian_rel_action_server
  cartesian_rel_action_client
  cartesian_rel_action_client_test
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
