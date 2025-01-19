<!--
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-06 10:48:02
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-06 10:48:28
 * @FilePath: /planning_control_node/src/obstacle_node/README.md
 * @Description: obstacle_service 服务调用示例
-->
# 添加一个箱体障碍物
ros2 service call /manageObstacle obstacle_node/srv/ManageObstacle "{
  operation: 1,
  obstacle_id: 'custom_box_1',
  primitive_type: 1,
  dimensions: [0.2, 0.2, 0.3],
  pose: {
    position: {x: 0.4, y: 0.3, z: 0.15},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  }
}"

# 添加一个圆柱体障碍物
ros2 service call /manageObstacle obstacle_node/srv/ManageObstacle "{
  operation: 1,
  obstacle_id: 'custom_cylinder_1',
  primitive_type: 2,
  dimensions: [0.1, 0.3],
  pose: {
    position: {x: 0.5, y: -0.2, z: 0.15},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  }
}"

# 添加一个平面障碍物
ros2 service call /manageObstacle obstacle_node/srv/ManageObstacle "{
  operation: 1,
  obstacle_id: 'custom_plane_1',
  primitive_type: 3,
  dimensions: [1.0, 0.0, 0.0, -0.5],
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}}
}"

# 移除一个障碍物
ros2 service call /manageObstacle obstacle_node/srv/ManageObstacle "{
  operation: 2, obstacle_id: 'custom_box_1'}"

# 清除所有障碍物
ros2 service call /manageObstacle obstacle_node/srv/ManageObstacle "{
  operation: 3
}"

# 查看服务的详细定义
ros2 interface show obstacle_node/srv/ManageObstacle