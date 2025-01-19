<!--
 * @Author: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @Date: 2025-01-03 17:00:51
 * @LastEditors: "feiyang_hong" "feiyang.hong@infinityrobot.cn"
 * @LastEditTime: 2025-01-03 17:04:08
 * @FilePath: /planning_control_node/src/planning_node/README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# 笛卡尔空间运动测试客户端

本文档描述了用于测试机械臂笛卡尔空间运动的两个测试客户端程序。

## 功能概述

这两个测试客户端分别用于测试：
1. 笛卡尔空间绝对位置运动 (`cartesian_abs_action_client_test`)
2. 笛卡尔空间相对位置运动 (`cartesian_rel_action_client_test`)

每个测试客户端都包含多个预定义的测试用例，用于验证机械臂的运动功能和避障能力。

## 测试用例说明

### 绝对位置运动测试
包含以下测试用例：
1. 基本位置移动
   - 初始位置 (0.4, 0.0, 0.5)
   - 前方位置 (0.5, 0.2, 0.3)
   - 后方位置 (0.4, -0.2, 0.3)
   - 返回初始位置 (0.4, 0.0, 0.5)

2. 避障测试
   - 避障可执行位置 (0.5, 0.0, 0.6)
   - 避障不可执行位置 (0.5, 0.0, -0.1)
   - 避障可执行位置 (0.5, 0.3, 0.6)

### 相对位置运动测试
包含以下测试用例：
1. 基本相对运动
   - 向前移动 10cm (+0.1, 0.0, 0.0)
   - 向右移动 10cm (0.0, +0.1, 0.0)
   - 向上移动 10cm (0.0, 0.0, +0.1)
   - 返回起点 (-0.1, -0.1, -0.1)

2. 复合相对运动
   - 斜向前右移动 (+0.1, +0.1, 0.0)
   - 斜向后右上移动 (-0.1, +0.1, +0.1)
   - 返回起点 (0.0, -0.2, -0.1)

## 使用方法

# 在工作空间根目录下
colcon build --packages-select planning_node
source install/setup.bash


### 运行测试
1. 首先启动机械臂和相关节点

# 在终端1中启动机械臂
ros2 launch moveit_node demo_test.launch.py

2. 运行绝对位置测试
ros2 run planning_node cartesian_abs_action_client_test

3. 运行相对位置测试
ros2 run planning_node cartesian_rel_action_client_test

## 测试结果说明

测试客户端会实时显示以下信息：
- 当前测试用例的名称和进度
- 每个目标位置的执行状态
- 实时位置反馈和完成百分比
- 测试结果（成功/失败）及相关信息

## 示例输出：
[INFO] [timestamp] [test_client]: 开始执行测试用例1: 基本位置移动
[INFO] [timestamp] [test_client]: 执行移动: 初始位置 (0.40, 0.00, 0.50)
[INFO] [timestamp] [test_client]: 当前位置: (0.35, 0.00, 0.45), 进度: 50.0%
[INFO] [timestamp] [test_client]: 目标位置 初始位置 到达成功: 已到达目标位置


## 注意事项

1. 运行测试前确保：
   - 机械臂已正确启动
   - 工作空间无障碍物
   - 机械臂处于安全位置

2. 安全考虑：
   - 测试过程中保持警惕
   - 准备好紧急停止按钮
   - 确保机械臂工作范围内无人

3. 错误处理：
   - 如果测试过程中出现错误，程序会自动停止并显示错误信息
   - 检查错误日志以确定问题原因

## 自定义测试用例

如需添加新的测试用例，可以修改源代码中的 `test_cases_` 初始化部分：
TestCase new_test_case;
new_test_case.name = "新测试用例名称";
new_test_case.descriptions = {"位置1描述", "位置2描述", ...};
new_test_case.targets.push_back(create_target(x1, y1, z1));
new_test_case.targets.push_back(create_target(x2, y2, z2));
test_cases_.push_back(new_test_case);
