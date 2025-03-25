import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob

# 读取所有测试结果文件
rrt_files = glob.glob('src/planning_node/test_results/KDL_RRTConnect_test_results_*.csv')
bitrrt_files = glob.glob('src/planning_node/test_results/TRAC_IK_BiTRRT_test_results_*.csv')  # 修正文件名

print("找到的RRT文件:", rrt_files)
print("找到的BiTRRT文件:", bitrrt_files)

# 存储数据
rrt_data = []
bitrrt_data = []

# 读取RRTConnect数据
for file in rrt_files:
    df = pd.read_csv(file)
    # 只读取到Statistics行之前的数据
    df = df[df['Position'].astype(str).str.isnumeric()]
    rrt_data.append({
        'planning_time': df['Planning Time (s)'].values,
        'path_length': df['Path Length (m)'].values,
        'joint_movement': df['Joint Movement (rad)'].values,
        'smoothness': df['Smoothness'].values
    })

# 读取BiTRRT数据
for file in bitrrt_files:
    df = pd.read_csv(file)
    # 只读取到Statistics行之前的数据
    df = df[df['Position'].astype(str).str.isnumeric()]
    bitrrt_data.append({
        'planning_time': df['Planning Time (s)'].values,
        'path_length': df['Path Length (m)'].values,
        'joint_movement': df['Joint Movement (rad)'].values,
        'smoothness': df['Smoothness'].values
    })
print(rrt_data)
# 创建4x2的折线图
fig, axes = plt.subplots(4, 2, figsize=(15, 20))
metrics = ['planning_time', 'path_length', 'joint_movement', 'smoothness']
titles = ['Planning Time (s)', 'Path Length (m)', 'Joint Movement (rad)', 'Smoothness']

# 绘制折线图
for i, (metric, title) in enumerate(zip(metrics, titles)):
    # 计算当前指标的最大值和最小值
    all_values = []
    for data in rrt_data + bitrrt_data:
        all_values.extend(data[metric])
    y_min = min(all_values)
    # y_min = 0
    y_max = max(all_values)
    
    # RRTConnect
    ax = axes[i, 0]
    for j in range(len(rrt_data)):
        ax.plot(rrt_data[j][metric], label=f'Test {j+1}')
    ax.set_title(f'RRTConnect - {title}', fontsize=16)
    ax.set_xlabel('Position', fontsize=16)
    ax.set_ylabel(title, fontsize=16)
    ax.tick_params(axis='both', labelsize=16)  # 设置刻度字体大小
    ax.set_ylim(y_min, y_max)  # 设置y轴范围
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=16)  # 指定图例位置在右上角

    # BiTRRT
    ax = axes[i, 1]
    for j in range(len(bitrrt_data)):
        ax.plot(bitrrt_data[j][metric], label=f'Test {j+1}')
    ax.set_title(f'BiTRRT - {title}', fontsize=16)
    ax.set_xlabel('Position', fontsize=16)
    ax.set_ylabel(title, fontsize=16)
    ax.tick_params(axis='both', labelsize=16)  # 设置刻度字体大小
    ax.set_ylim(y_min, y_max)  # 设置相同的y轴范围
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=16)  # 指定图例位置在右上角

plt.tight_layout()
plt.savefig('src/planning_node/test_results/comparison_lines.png')
plt.show()
plt.close()

# 计算平均值并创建柱状图
rrt_means = {
    'Planning Time (s)': np.mean([np.mean(d['planning_time']) for d in rrt_data]),
    'Path Length (m)': np.mean([np.mean(d['path_length']) for d in rrt_data]),
    'Joint Movement (rad)': np.mean([np.mean(d['joint_movement']) for d in rrt_data]),
    'Smoothness': np.mean([np.mean(d['smoothness']) for d in rrt_data])
}

bitrrt_means = {
    'Planning Time (s)': np.mean([np.mean(d['planning_time']) for d in bitrrt_data]),
    'Path Length (m)': np.mean([np.mean(d['path_length']) for d in bitrrt_data]),
    'Joint Movement (rad)': np.mean([np.mean(d['joint_movement']) for d in bitrrt_data]),
    'Smoothness': np.mean([np.mean(d['smoothness']) for d in bitrrt_data])
}

print("RRT平均值:", rrt_means)
print("BiTRRT平均值:", bitrrt_means)

# 创建柱状图
plt.figure(figsize=(12, 6))
x = np.arange(len(titles))
width = 0.35

# 绘制柱状图
plt.bar(x - width/2, list(rrt_means.values()), width, label='RRTConnect', color='skyblue')
plt.bar(x + width/2, list(bitrrt_means.values()), width, label='BiTRRT', color='lightcoral')

# 设置图表属性
plt.ylabel('Values', fontsize=16)
plt.title('Average Metrics Comparison', fontsize=16)
plt.xticks(x, titles, fontsize=16)
plt.yticks(fontsize=16)  # 设置y轴刻度字体大小
plt.legend(fontsize=16)

# 在柱子上添加数值标签
for i, v in enumerate(list(rrt_means.values())):
    plt.text(i - width/2, v, f'{v:.3f}', ha='center', va='bottom', fontsize=16)
for i, v in enumerate(list(bitrrt_means.values())):
    plt.text(i + width/2, v, f'{v:.3f}', ha='center', va='bottom', fontsize=16)

plt.tight_layout()
plt.savefig('src/planning_node/test_results/comparison_bars.png')
plt.show()
plt.close()