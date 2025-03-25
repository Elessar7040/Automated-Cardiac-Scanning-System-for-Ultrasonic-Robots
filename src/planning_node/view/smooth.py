import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os

# Read all test result files
result_files = glob.glob('src/planning_node/test_results/smoothing_test_results_*.csv')

print("Found result files:", result_files)

# Store data
no_smooth_data = []
cubic_data = []
bspline_data = []

# Read data
for file in result_files:
    df = pd.read_csv(file)
    
    # Separate data, only read up to the "平均值" row
    data_rows = df[~df['Position'].astype(str).str.contains('平均值')]
    
    # Convert numeric columns to float
    numeric_columns = ['Planning Time (s)', 'Smoothing Time (s)', 'Total Time (s)', 
                      'Path Length (m)', 'Joint Movement (rad)', 'Smoothness']
    for col in numeric_columns:
        if col in data_rows.columns:
            data_rows[col] = pd.to_numeric(data_rows[col], errors='coerce')
    
    # Group by method
    no_smooth_rows = data_rows[data_rows['Method'] == '无平滑']
    cubic_rows = data_rows[data_rows['Method'] == '三次多项式']
    bspline_rows = data_rows[data_rows['Method'] == 'B样条曲线']
    
    # Store data
    no_smooth_data.append({
        'planning_time': no_smooth_rows['Planning Time (s)'].values,
        'total_time': no_smooth_rows['Total Time (s)'].values,
        'path_length': no_smooth_rows['Path Length (m)'].values,
        'joint_movement': no_smooth_rows['Joint Movement (rad)'].values,
        'smoothness': no_smooth_rows['Smoothness'].values,
    })
    
    cubic_data.append({
        'planning_time': cubic_rows['Planning Time (s)'].values,
        'smoothing_time': cubic_rows['Smoothing Time (s)'].values,
        'total_time': cubic_rows['Total Time (s)'].values,
        'path_length': cubic_rows['Path Length (m)'].values,
        'joint_movement': cubic_rows['Joint Movement (rad)'].values,
        'smoothness': cubic_rows['Smoothness'].values,
    })
    
    bspline_data.append({
        'planning_time': bspline_rows['Planning Time (s)'].values,
        'smoothing_time': bspline_rows['Smoothing Time (s)'].values,
        'total_time': bspline_rows['Total Time (s)'].values,
        'path_length': bspline_rows['Path Length (m)'].values,
        'joint_movement': bspline_rows['Joint Movement (rad)'].values,
        'smoothness': bspline_rows['Smoothness'].values,
    })

# Create 4x3 line plots
fig, axes = plt.subplots(4, 3, figsize=(18, 20))
metrics = ['total_time', 'path_length', 'joint_movement', 'smoothness']
titles = ['Total Time (s)', 'Path Length (m)', 'Joint Movement (rad)', 'Smoothness']
method_titles = ['No Smoothing', 'Cubic Polynomial', 'B-Spline']

# Draw line plots
for i, (metric, title) in enumerate(zip(metrics, titles)):
    # Calculate min and max values for current metric across all methods
    all_values = []
    for data in no_smooth_data + cubic_data + bspline_data:
        if metric in data and len(data[metric]) > 0:
            all_values.extend(data[metric])
    
    if all_values:
        y_min = min(all_values)
        y_max = max(all_values)
    else:
        y_min, y_max = 0, 1  # Default if no data
    
    # No smoothing (first column)
    ax = axes[i, 0]
    for j, data in enumerate(no_smooth_data):
        if metric in data and len(data[metric]) > 0:
            ax.plot(data[metric], label=f'Test {j+1}')
    ax.set_title(f'{method_titles[0]} - {title}', fontsize=14)
    ax.set_xlabel('Position', fontsize=12)
    ax.set_ylabel(title, fontsize=12)
    ax.tick_params(axis='both', labelsize=10)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=10)
    
    # Cubic polynomial (second column)
    ax = axes[i, 1]
    for j, data in enumerate(cubic_data):
        if metric in data and len(data[metric]) > 0:
            ax.plot(data[metric], label=f'Test {j+1}')
    ax.set_title(f'{method_titles[1]} - {title}', fontsize=14)
    ax.set_xlabel('Position', fontsize=12)
    ax.set_ylabel(title, fontsize=12)
    ax.tick_params(axis='both', labelsize=10)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=10)
    
    # B-spline (third column)
    ax = axes[i, 2]
    for j, data in enumerate(bspline_data):
        if metric in data and len(data[metric]) > 0:
            ax.plot(data[metric], label=f'Test {j+1}')
    ax.set_title(f'{method_titles[2]} - {title}', fontsize=14)
    ax.set_xlabel('Position', fontsize=12)
    ax.set_ylabel(title, fontsize=12)
    ax.tick_params(axis='both', labelsize=10)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=10)

plt.tight_layout()
plt.savefig('src/planning_node/test_results/smoothing_comparison_lines.png', dpi=300)
plt.show()
plt.close()

# 计算平均值并创建柱状图
# 从每个文件中读取平均值
avg_data = {
    'no_smooth': {'total_time': [], 'path_length': [], 'joint_movement': [], 'smoothness': []},
    'cubic': {'total_time': [], 'path_length': [], 'joint_movement': [], 'smoothness': []},
    'bspline': {'total_time': [], 'path_length': [], 'joint_movement': [], 'smoothness': []}
}

for file in result_files:
    df = pd.read_csv(file)
    
    # 找到平均值行
    avg_rows = df[df['Position'].astype(str).str.contains('Statistics', na=False)]
    if len(avg_rows) == 0:
        continue
    
    # 获取平均值行之后的数据
    avg_data_rows = df.iloc[df[df['Position'].astype(str).str.contains('Statistics', na=False)].index[0]+1:]
    
    # 确保数值列是数值类型
    numeric_columns = ['Planning Time (s)', 'Smoothing Time (s)', 'Total Time (s)', 
                      'Path Length (m)', 'Joint Movement (rad)', 'Smoothness']
    # for col in numeric_columns:
    #     if col in avg_data_rows.columns:
    #         avg_data_rows[col] = pd.to_numeric(avg_data_rows[col], errors='coerce')
    
    # 按方法分组
    no_smooth_avg = avg_data_rows[avg_data_rows['Method'] == '无平滑']
    cubic_avg = avg_data_rows[avg_data_rows['Method'] == '三次多项式']
    bspline_avg = avg_data_rows[avg_data_rows['Method'] == 'B样条曲线']
    
    # 存储平均值数据
    if not no_smooth_avg.empty:
        avg_data['no_smooth']['total_time'].append(float(no_smooth_avg['Total Time (s)'].values[0]))
        avg_data['no_smooth']['path_length'].append(float(no_smooth_avg['Path Length (m)'].values[0]))
        avg_data['no_smooth']['joint_movement'].append(float(no_smooth_avg['Joint Movement (rad)'].values[0]))
        avg_data['no_smooth']['smoothness'].append(float(no_smooth_avg['Smoothness'].values[0]))
    
    if not cubic_avg.empty:
        avg_data['cubic']['total_time'].append(float(cubic_avg['Total Time (s)'].values[0]))
        avg_data['cubic']['path_length'].append(float(cubic_avg['Path Length (m)'].values[0]))
        avg_data['cubic']['joint_movement'].append(float(cubic_avg['Joint Movement (rad)'].values[0]))
        avg_data['cubic']['smoothness'].append(float(cubic_avg['Smoothness'].values[0]))
    
    if not bspline_avg.empty:
        avg_data['bspline']['total_time'].append(float(bspline_avg['Total Time (s)'].values[0]))
        avg_data['bspline']['path_length'].append(float(bspline_avg['Path Length (m)'].values[0]))
        avg_data['bspline']['joint_movement'].append(float(bspline_avg['Joint Movement (rad)'].values[0]))
        avg_data['bspline']['smoothness'].append(float(bspline_avg['Smoothness'].values[0]))

# 计算所有测试的平均值
final_avg = {
    'no_smooth': {
        'total_time': np.mean(avg_data['no_smooth']['total_time']),
        'path_length': np.mean(avg_data['no_smooth']['path_length']),
        'joint_movement': np.mean(avg_data['no_smooth']['joint_movement']),
        'smoothness': np.mean(avg_data['no_smooth']['smoothness']),
    },
    'cubic': {
        'total_time': np.mean(avg_data['cubic']['total_time']),
        'path_length': np.mean(avg_data['cubic']['path_length']),
        'joint_movement': np.mean(avg_data['cubic']['joint_movement']),
        'smoothness': np.mean(avg_data['cubic']['smoothness']),
    },
    'bspline': {
        'total_time': np.mean(avg_data['bspline']['total_time']),
        'path_length': np.mean(avg_data['bspline']['path_length']),
        'joint_movement': np.mean(avg_data['bspline']['joint_movement']),
        'smoothness': np.mean(avg_data['bspline']['smoothness']),
    }
}

# 创建柱状图
metrics_for_bar = ['total_time', 'path_length', 'joint_movement', 'smoothness']
titles_for_bar = ['Total Time (s)', 'Path Length (m)', 'Joint Movement (rad)', 'Smoothness']

plt.figure(figsize=(15, 10))
x = np.arange(len(metrics_for_bar))
width = 0.25

# Draw bar chart
plt.bar(x - width, [final_avg['no_smooth'][m] for m in metrics_for_bar], width, label='No Smoothing', color='skyblue')
plt.bar(x, [final_avg['cubic'][m] for m in metrics_for_bar], width, label='Cubic Polynomial', color='lightcoral')
plt.bar(x + width, [final_avg['bspline'][m] for m in metrics_for_bar], width, label='B-Spline', color='lightgreen')

# 设置图表属性
plt.ylabel('Values', fontsize=16)
plt.title('Average Metrics Comparison', fontsize=16)
plt.xticks(x, titles_for_bar, fontsize=16)
plt.yticks(fontsize=16)  # 设置y轴刻度字体大小
plt.legend(fontsize=16)

# 在柱子上添加数值标签
for i, v in enumerate([final_avg['no_smooth'][m] for m in metrics_for_bar]):
    plt.text(i - width, v, f'{v:.3f}', ha='center', va='bottom', fontsize=12)
for i, v in enumerate([final_avg['cubic'][m] for m in metrics_for_bar]):
    plt.text(i, v, f'{v:.3f}', ha='center', va='bottom', fontsize=12)
for i, v in enumerate([final_avg['bspline'][m] for m in metrics_for_bar]):
    plt.text(i + width, v, f'{v:.3f}', ha='center', va='bottom', fontsize=12)

plt.tight_layout()
plt.savefig('src/planning_node/test_results/smoothing_comparison_bars.png', dpi=300)
plt.show()
plt.close()

# 创建单独的柱状图，用于比较总时间、路径长度和关节移动量
plt.figure(figsize=(12, 6))
x = np.arange(3)
width = 0.25

metrics_subset = ['total_time', 'path_length', 'joint_movement']
titles_subset = ['Total Time (s)', 'Path Length (m)', 'Joint Movement (rad)']

# Draw bar chart
plt.bar(x - width, [final_avg['no_smooth'][m] for m in metrics_subset], width, label='No Smoothing', color='skyblue')
plt.bar(x, [final_avg['cubic'][m] for m in metrics_subset], width, label='Cubic Polynomial', color='lightcoral')
plt.bar(x + width, [final_avg['bspline'][m] for m in metrics_subset], width, label='B-Spline', color='lightgreen')

# 设置图表属性
plt.ylabel('Values', fontsize=16)
plt.title('Performance Metrics Comparison', fontsize=16)
plt.xticks(x, titles_subset, fontsize=16)
plt.yticks(fontsize=16)  # 设置y轴刻度字体大小
plt.legend(fontsize=16)

# 在柱子上添加数值标签
for i, v in enumerate([final_avg['no_smooth'][m] for m in metrics_subset]):
    plt.text(i - width, v, f'{v:.3f}', ha='center', va='bottom', fontsize=12)
for i, v in enumerate([final_avg['cubic'][m] for m in metrics_subset]):
    plt.text(i, v, f'{v:.3f}', ha='center', va='bottom', fontsize=12)
for i, v in enumerate([final_avg['bspline'][m] for m in metrics_subset]):
    plt.text(i + width, v, f'{v:.3f}', ha='center', va='bottom', fontsize=12)

plt.tight_layout()
plt.savefig('src/planning_node/test_results/performance_comparison_bars.png', dpi=300)
plt.show()
plt.close()

print("Visualization complete, charts saved to src/planning_node/test_results/ directory")

