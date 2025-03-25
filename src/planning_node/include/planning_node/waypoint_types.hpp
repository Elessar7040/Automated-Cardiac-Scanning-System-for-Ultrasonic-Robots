/*
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 14:21:57
 * @LastEditors: “feiyang_hong” “feiyang.hong@infinityrobot.cn”
 * @LastEditTime: 2025-01-08 17:26:27
 * @FilePath: /planning_control_node/src/planning_node/include/planning_node/waypoint_types.hpp
 * @Description: waypoint 的类型定义
 */
#ifndef WAYPOINT_TYPES_HPP_
#define WAYPOINT_TYPES_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace planning_node
{
struct WaypointAction {
    static constexpr uint8_t ACTION_ALGORITHM = 0x01;    // 启动算法
    static constexpr uint8_t ACTION_END_CONTROL = 0x02;  // 末端控制
    uint8_t action_type;
    // 算法模式
    static constexpr uint8_t ALG_RECOGNITION = 0x01;     // 识别算法
    uint8_t algorithm_mode;
    // 末端执行器
    static constexpr uint8_t END_SUCTION = 0x01;        // 吸盘
    static constexpr uint8_t END_PROBE = 0x02;        // 探针
    uint8_t end_effector_type;
    uint32_t end_effector_id;                           // 末端夹具型号
    float end_effector_action;                          // 末端夹具动作
    // 设备控制
    static constexpr uint8_t EQUIP_BENDER = 0x01;       // 折弯机
    static constexpr uint8_t EQUIP_PUNCH = 0x02;        // 冲压机
    uint8_t equipment_type;
    uint32_t equipment_id;                              // 设备ID
};

struct Waypoint {
    // 锚点类型
    static constexpr uint8_t TYPE_INIT = 0x01;      // 初始点
    static constexpr uint8_t TYPE_WORK = 0x02;      // 工作点
    static constexpr uint8_t TYPE_END = 0x03;       // 结束点
    uint8_t waypoint_type;

    // 锚点工作模式
    static constexpr uint8_t MODE_PICK = 0x01;      // 取料
    static constexpr uint8_t MODE_LOAD = 0x02;      // 上料
    static constexpr uint8_t MODE_UNLOAD = 0x03;    // 下料
    static constexpr uint8_t MODE_STACK = 0x04;     // 码垛
    uint8_t waypoint_mode;

    // 航点位姿
    geometry_msgs::msg::Pose waypoint_pose;

    // 航点规划模式
    static constexpr uint8_t PLAN_STOP = 0x01;      // 到点停止
    static constexpr uint8_t PLAN_SMOOTH = 0x02;    // 平滑过渡
    static constexpr uint8_t PLAN_FORCE = 0x03;     // 力反馈停止
    uint8_t waypoint_plan_mode;

    // 航点行为
    WaypointAction waypoint_action;
};

} // namespace planning_node

#endif // WAYPOINT_TYPES_HPP_ 