cmake_minimum_required(VERSION 3.8)
project(planning_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(moveit_example_test 
  src/moveit_example_test.cpp
)
add_executable(moveit_cartesian_test_1 
  src/moveit_cartesian_test_1.cpp
)
add_executable(moveit_cartesian_test_2 
  src/moveit_cartesian_test_2.cpp
)
add_executable(moveit_cartesian_test_3 
  src/moveit_cartesian_test_3.cpp
)
add_executable(pose_publisher 
  src/pose_publisher.cpp
)
add_executable(moveit_obstacle_avoid_1 
  src/moveit_obstacle_avoid_1.cpp
)
add_executable(match_point_converter 
  src/match_point_converter.cpp
)

ament_target_dependencies(moveit_example_test
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)
ament_target_dependencies(moveit_cartesian_test_1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)
ament_target_dependencies(moveit_cartesian_test_2
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
)
ament_target_dependencies(pose_publisher
  rclcpp
  geometry_msgs
)
ament_target_dependencies(moveit_cartesian_test_3
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
  tf2_ros
  sensor_msgs
)
ament_target_dependencies(moveit_obstacle_avoid_1
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  moveit_msgs
)
ament_target_dependencies(match_point_converter
  rclcpp
  geometry_msgs
)

install(TARGETS
  moveit_cartesian_test_1
  moveit_cartesian_test_2
  moveit_cartesian_test_3
  moveit_example_test
  pose_publisher
  moveit_obstacle_avoid_1
  match_point_converter
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
