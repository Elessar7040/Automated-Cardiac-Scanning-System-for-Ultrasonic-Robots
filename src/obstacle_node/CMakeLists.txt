cmake_minimum_required(VERSION 3.8)
project(obstacle_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)

# 添加缺失的srdfdom和geometric_shapes依赖
find_package(srdfdom REQUIRED)
find_package(geometric_shapes REQUIRED)

# 添加action依赖
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ManageObstacle.srv"
  "srv/ManageMaterial.srv"
  DEPENDENCIES geometry_msgs
)
# 生成导出依赖
rosidl_get_typesupport_target(cpp_typesupport_target 
  "${PROJECT_NAME}" "rosidl_typesupport_cpp"
)

add_executable(obstacle_test_1 
  src/obstacle_test_1.cpp)
add_executable(obstacle_service 
  src/obstacle_service.cpp)

ament_target_dependencies(obstacle_test_1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2_ros
  eigen_stl_containers
  Eigen3
  moveit_core
  srdfdom
  geometric_shapes
)
ament_target_dependencies(obstacle_service
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2_ros
  eigen_stl_containers
  Eigen3
  moveit_core
  srdfdom
  geometric_shapes
)

# 链接生成的接口库
target_link_libraries(obstacle_test_1 
  "${cpp_typesupport_target}"
)
target_link_libraries(obstacle_service 
  "${cpp_typesupport_target}"
)

install(TARGETS
  obstacle_test_1
  obstacle_service
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
