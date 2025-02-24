'''
Author: feiyang feiyang.hong@infinityrobot.cn
Date: 2025-02-24 16:16:46
LastEditors: feiyang feiyang.hong@infinityrobot.cn
LastEditTime: 2025-02-24 16:49:56
FilePath: /IR100/src/planning_control_node/src/planning_node/view/ik.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # 避免Qt相关的错误
import matplotlib.pyplot as plt
import numpy as np
import glob

# 读取所有测试结果文件
kdl_files = glob.glob('src/planning_node/test_results/KDL_*_test_results_*.csv')
trac_files = glob.glob('src/planning_node/test_results/TRAC_IK_*_test_results_*.csv')

print("找到的KDL文件:", kdl_files)
print("找到的TRAC_IK文件:", trac_files)

# 读取数据函数
def read_ik_times_from_files(files):
    data = []
    for file in files:
        df = pd.read_csv(file)
        df = df[df['Position'].astype(str).str.isnumeric()]
        data.append(df['IK Time (s)'].values.astype(float))  # 确保数据类型为float
    return data

# 读取所有数据
kdl_times = read_ik_times_from_files(kdl_files)
trac_times = read_ik_times_from_files(trac_files)

# 创建一个包含两个子图的图表
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 8))

# 找到所有数据的最大值和最小值，用于统一y轴范围
all_times = np.concatenate([np.concatenate(kdl_times), np.concatenate(trac_times)])
y_min = np.min(all_times)
y_max = np.max(all_times)

# 绘制KDL数据
for i, times in enumerate(kdl_times):
    ax1.plot(times, label=f'Test {i+1}')
ax1.set_title('KDL IK Solving Time')
ax1.set_xlabel('Position')
ax1.set_ylabel('Time (s)')
ax1.grid(True)
ax1.legend()
ax1.set_ylim(y_min, y_max)

# 绘制TRAC_IK数据
for i, times in enumerate(trac_times):
    ax2.plot(times, label=f'Test {i+1}')
ax2.set_title('TRAC_IK Solving Time')
ax2.set_xlabel('Position')
ax2.set_ylabel('Time (s)')
ax2.grid(True)
ax2.legend()
ax2.set_ylim(y_min, y_max)

plt.tight_layout()
plt.savefig('src/planning_node/test_results/kdl_trac_time_comparison.png')
plt.close()

# 计算平均求解时间
kdl_mean = np.mean([np.mean(times) for times in kdl_times])
trac_mean = np.mean([np.mean(times) for times in trac_times])

print("\n平均求解时间统计:")
print(f"KDL平均求解时间: {kdl_mean:.8f} 秒")
print(f"TRAC_IK平均求解时间: {trac_mean:.8f} 秒")

# 创建柱状图比较平均求解时间
plt.figure(figsize=(8, 6))
plt.bar(['KDL', 'TRAC_IK'], [kdl_mean, trac_mean], color=['skyblue', 'lightcoral'])
plt.title('Average IK Solving Time')
plt.ylabel('Time (s)')

# 在柱子上添加数值标签
for i, v in enumerate([kdl_mean, trac_mean]):
    plt.text(i, v, f'{v:.8f}', ha='center', va='bottom')

plt.grid(True, axis='y')
plt.tight_layout()
plt.savefig('src/planning_node/test_results/ik_time_average.png')
plt.close()

