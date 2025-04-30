cmake_minimum_required(VERSION 3.8)
project(pcl_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 启用 CMP0074 策略
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# 查找Boost包
find_package(Boost REQUIRED COMPONENTS system filesystem date_time)

# set(PCL_DIR "/usr/local/share/pcl-1.14")

# 查找所需的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

# 添加可执行文件
add_executable(pointcloud_transform_node 
  src/pointcloud_transform.cpp
)
add_executable(pointcloud_normal_estimator_node 
  src/pointcloud_normal_estimator.cpp
)
add_executable(voxel_size_test
  test/voxel_size_test.cpp
)
add_executable(normal_radius_test
  test/normal_radius_test.cpp
)
add_executable(view_point_cloud_normals test/view_point_cloud_normals.cpp)
target_link_libraries(view_point_cloud_normals ${PCL_LIBRARIES})

# 包含目录
target_include_directories(pointcloud_transform_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_include_directories(pointcloud_normal_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_include_directories(voxel_size_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_include_directories(normal_radius_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
# 设置编译特性
target_compile_features(pointcloud_transform_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(pointcloud_normal_estimator_node PUBLIC c_std_99 cxx_std_17)
target_compile_features(voxel_size_test PUBLIC c_std_99 cxx_std_17)
target_compile_features(normal_radius_test PUBLIC c_std_99 cxx_std_17)
# 链接库
target_link_libraries(pointcloud_transform_node
  ${PCL_LIBRARIES}  
  ${pcl_ros_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(pointcloud_normal_estimator_node
  ${PCL_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(voxel_size_test
  ${PCL_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(normal_radius_test
  ${PCL_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 添加依赖
ament_target_dependencies(pointcloud_transform_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  geometry_msgs
  pcl_ros
)
ament_target_dependencies(pointcloud_normal_estimator_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  geometry_msgs
  pcl_ros
)
ament_target_dependencies(voxel_size_test
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  geometry_msgs
  pcl_ros
)
ament_target_dependencies(normal_radius_test
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  geometry_msgs
  pcl_ros
)
# 安装目标
install(TARGETS pointcloud_transform_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS pointcloud_normal_estimator_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS voxel_size_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS normal_radius_test
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS view_point_cloud_normals
  DESTINATION lib/${PROJECT_NAME})

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

# 导出包信息
ament_package()
