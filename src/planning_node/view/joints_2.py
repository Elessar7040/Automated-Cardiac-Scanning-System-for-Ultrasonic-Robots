import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# 读取CSV文件
df = pd.read_csv('src/planning_node/test_results/arm_trajectory.csv')

# 获取所有关节名称
joint_names = df['joint_name'].unique()

# 创建图表
fig, axes = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
plt.subplots_adjust(hspace=0.3)

# 为每个关节设置不同的颜色
colors = plt.cm.tab10(np.linspace(0, 1, len(joint_names)))

# 绘制actual_position
for i, joint in enumerate(joint_names):
    joint_data = df[df['joint_name'] == joint]
    axes[0].plot(joint_data['timestamp'].to_numpy(), joint_data['actual_position'].to_numpy(), 
                 color=colors[i], label=joint)
axes[0].set_ylabel('实际位置 (rad)')
axes[0].set_title('关节实际位置随时间变化')
axes[0].grid(True)
axes[0].legend(loc='best')

# 绘制actual_velocity
for i, joint in enumerate(joint_names):
    joint_data = df[df['joint_name'] == joint]
    axes[1].plot(joint_data['timestamp'].to_numpy(), joint_data['actual_velocity'].to_numpy(), 
                 color=colors[i], label=joint)
axes[1].set_ylabel('实际速度 (rad/s)')
axes[1].set_title('关节实际速度随时间变化')
axes[1].grid(True)
axes[1].legend(loc='best')

# 绘制desired_acceleration
for i, joint in enumerate(joint_names):
    joint_data = df[df['joint_name'] == joint]
    axes[2].plot(joint_data['timestamp'].to_numpy(), joint_data['desired_acceleration'].to_numpy(), 
                 color=colors[i], label=joint)
axes[2].set_xlabel('时间 (s)')
axes[2].set_ylabel('期望加速度 (rad/s²)')
axes[2].set_title('关节期望加速度随时间变化')
axes[2].grid(True)
axes[2].legend(loc='best')

# 保存图表
plt.savefig('src/planning_node/test_results/joint_trajectory_data.png', dpi=300, bbox_inches='tight')
print(f"图表已保存为 'joint_trajectory_data.png'")

# 显示图表
plt.show()