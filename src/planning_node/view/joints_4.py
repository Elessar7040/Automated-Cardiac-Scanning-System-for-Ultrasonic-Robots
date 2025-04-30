import pandas as pd
import numpy as np

# 读取CSV文件
print("读取joint_states.csv文件...")
df = pd.read_csv('src/planning_node/test_results/joint_states.csv')

# 处理NaN值
df['effort'] = df['effort'].fillna(0)

# 获取所有关节名称
joint_names = df['joint_name'].unique()
print(f"检测到的关节: {joint_names}")

# 创建一个新的DataFrame存储结果
result_df = pd.DataFrame()

# 对每个关节分别处理
for joint in joint_names:
    print(f"处理关节: {joint}")
    # 获取当前关节的数据，按时间戳排序
    joint_data = df[df['joint_name'] == joint].sort_values('timestamp')
    
    # 计算加速度：速度的差分除以时间的差分
    # 先将数据转换为numpy数组，便于操作
    timestamps = joint_data['timestamp'].to_numpy()
    velocities = joint_data['velocity'].to_numpy()
    
    # 初始化加速度数组，与原始数组大小相同
    accelerations = np.zeros_like(velocities)
    
    # 计算加速度（中间点使用中心差分法，首末点使用前向/后向差分法）
    for i in range(1, len(velocities) - 1):
        # 中心差分: (v[i+1] - v[i-1]) / (t[i+1] - t[i-1])
        dt = timestamps[i+1] - timestamps[i-1]
        if dt > 0:  # 避免除以零
            accelerations[i] = (velocities[i+1] - velocities[i-1]) / dt
    
    # 处理首点和末点
    if len(velocities) > 1:
        # 首点：前向差分
        dt = timestamps[1] - timestamps[0]
        if dt > 0:
            accelerations[0] = (velocities[1] - velocities[0]) / dt
        
        # 末点：后向差分
        dt = timestamps[-1] - timestamps[-2]
        if dt > 0:
            accelerations[-1] = (velocities[-1] - velocities[-2]) / dt
    
    # 将计算得到的加速度添加到原始数据中
    joint_data['acceleration'] = accelerations
    
    # 添加到结果DataFrame
    result_df = pd.concat([result_df, joint_data])

# 按时间戳排序
result_df = result_df.sort_values('timestamp')

# 保存为新文件
print("保存结果到joint_states_2.csv...")
result_df.to_csv('src/planning_node/test_results/joint_states_2.csv', index=False)

print("处理完成！数据已保存到joint_states_2.csv")

# 输出一些统计信息
print("\n数据统计:")
print(f"原始数据行数: {len(df)}")
print(f"处理后数据行数: {len(result_df)}")
print("\n加速度数据统计:")
print(result_df['acceleration'].describe())

# 可选：绘制图表查看加速度数据
try:
    import matplotlib.pyplot as plt
    
    # 为每个关节创建图表
    for joint in joint_names:
        joint_data = result_df[result_df['joint_name'] == joint]
        
        # 将数据转换为numpy数组以解决多维索引问题
        timestamps = joint_data['timestamp'].to_numpy()
        positions = joint_data['position'].to_numpy()
        velocities = joint_data['velocity'].to_numpy()
        accelerations = joint_data['acceleration'].to_numpy()
        
        plt.figure(figsize=(12, 8))
        
        plt.subplot(3, 1, 1)
        plt.plot(timestamps, positions)
        plt.title(f'{joint} - 位置')
        plt.grid(True)
        
        plt.subplot(3, 1, 2)
        plt.plot(timestamps, velocities)
        plt.title(f'{joint} - 速度')
        plt.grid(True)
        
        plt.subplot(3, 1, 3)
        plt.plot(timestamps, accelerations)
        plt.title(f'{joint} - 计算得到的加速度')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(f'src/planning_node/test_results/{joint}_with_acceleration.png')
        print(f"已创建图表: {joint}_with_acceleration.png")
    
    plt.close('all')  # 关闭所有图表
except ImportError:
    print("未安装matplotlib，跳过图表生成")