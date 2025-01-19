<!--
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-06 11:10:12
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-06 11:10:21
 * @FilePath: /planning_control_node/src/planning_node/include/planning_node/README.md
 * @Description: action_server 功能示例
-->
# 开启避障功能
ros2 service call /glob_planning_setting planning_node/srv/GlobPlanningSetting "{
  obstacle_avoidance_enabled: true,
  planning_mode: 0
}"

# 关闭避障功能
ros2 service call /glob_planning_setting planning_node/srv/GlobPlanningSetting "{
  obstacle_avoidance_enabled: false,
  planning_mode: 0
}"

# 设置不同的规划模式
# 普通模式 (mode 0)
ros2 service call /glob_planning_setting planning_node/srv/GlobPlanningSetting "{
  obstacle_avoidance_enabled: true,
  planning_mode: 0
}"

# 安全模式 (mode 1)
ros2 service call /glob_planning_setting planning_node/srv/GlobPlanningSetting "{
  obstacle_avoidance_enabled: true,
  planning_mode: 1
}"

# 快速模式 (mode 2)
ros2 service call /glob_planning_setting planning_node/srv/GlobPlanningSetting "{
  obstacle_avoidance_enabled: true,
  planning_mode: 2
}"

# 精确模式 (mode 3)
ros2 service call /glob_planning_setting planning_node/srv/GlobPlanningSetting "{
  obstacle_avoidance_enabled: true,
  planning_mode: 3
}"
