cmake_minimum_required(VERSION 3.8)
project(end_control_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 添加缺失的MoveIt依赖
find_package(geometric_shapes REQUIRED)
# find_package(srdfdom REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EndControl.srv"
  "action/MoveEndToRelPos.action"
  DEPENDENCIES geometry_msgs
)
# 生成导出依赖
rosidl_get_typesupport_target(cpp_typesupport_target 
  "${PROJECT_NAME}" "rosidl_typesupport_cpp"
)

# 添加可执行文件
add_executable(cartesian_rel_action_server 
  src/cartesian_rel_action_server.cpp)
add_executable(cartesian_rel_action_client 
  src/cartesian_rel_action_client.cpp)
add_executable(cartesian_rel_action_client_test 
  test/cartesian_rel_action_client_test.cpp)
add_executable(end_control_service 
  src/end_control_service.cpp)

# 添加依赖
ament_target_dependencies(cartesian_rel_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  eigen_stl_containers
  Eigen3
  geometry_msgs
  tf2_ros
  geometric_shapes
  # srdfdom
)

ament_target_dependencies(cartesian_rel_action_client
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  eigen_stl_containers
  Eigen3
  geometry_msgs
  geometric_shapes
  # srdfdom
)

ament_target_dependencies(cartesian_rel_action_client_test
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  eigen_stl_containers
  Eigen3
  geometry_msgs
  geometric_shapes
  # srdfdom
)

ament_target_dependencies(end_control_service
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  eigen_stl_containers
  Eigen3
  geometry_msgs
  sensor_msgs
  geometric_shapes
  # srdfdom
)

# 链接生成的接口库
target_link_libraries(cartesian_rel_action_server 
  "${cpp_typesupport_target}"
)
target_link_libraries(cartesian_rel_action_client 
  "${cpp_typesupport_target}"
)
target_link_libraries(cartesian_rel_action_client_test 
  "${cpp_typesupport_target}"
)
target_link_libraries(end_control_service 
  "${cpp_typesupport_target}"
)

# 安装可执行文件
install(TARGETS
  cartesian_rel_action_server
  cartesian_rel_action_client
  cartesian_rel_action_client_test
  end_control_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()