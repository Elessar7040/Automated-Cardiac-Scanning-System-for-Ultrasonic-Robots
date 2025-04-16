import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math

def plot_joint_data(csv_file):
    # 读取CSV文件
    df = pd.read_csv(csv_file)
    
    # 获取时间数据
    time = df['Time(s)'].values  # 转换为numpy数组
    
    # 提取关节名称（移除Position/Velocity/Acceleration后缀）
    joint_names = []
    for col in df.columns:
        if 'Position' in col:
            joint_name = col.split('_Position')[0]
            joint_names.append(joint_name)
    
    # 转换为角度制（弧度 -> 度）
    rad_to_deg = 180.0 / math.pi
    
    # 创建3个图表，分别用于角度、角速度和角加速度
    plt.figure(figsize=(18, 15))
    
    # 图1：关节角度
    plt.subplot(3, 1, 1)
    for joint in joint_names:
        position_col = f"{joint}_Position(rad)"
        # 将弧度转换为角度，并将Series转换为numpy数组
        plt.plot(time, df[position_col].values * rad_to_deg, label=joint)
    
    plt.title('Joints Position', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Angle (°)', fontsize=12)
    plt.grid(True)
    plt.legend(loc='upper right')
    
    # 图2：关节角速度
    plt.subplot(3, 1, 2)
    for joint in joint_names:
        velocity_col = f"{joint}_Velocity(rad/s)"
        # 将弧度/秒转换为度/秒，并将Series转换为numpy数组
        plt.plot(time, df[velocity_col].values * rad_to_deg, label=joint)
    
    plt.title('Joints Velocity', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Velocity (°/s)', fontsize=12)
    plt.grid(True)
    plt.legend(loc='upper right')
    
    # 图3：关节角加速度
    plt.subplot(3, 1, 3)
    for joint in joint_names:
        acceleration_col = f"{joint}_Acceleration(rad/s^2)"
        # 将弧度/秒^2转换为度/秒^2，并将Series转换为numpy数组
        plt.plot(time, df[acceleration_col].values * rad_to_deg, label=joint)
    
    plt.title('Joints Acceleration', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Acceleration (°/s²)', fontsize=12)
    plt.grid(True)
    plt.legend(loc='upper right')
    
    # 调整布局并保存图像
    plt.tight_layout()
    plt.savefig('src/planning_node/test_results/nurbs_joint_data/joint_trajectory_visualization.png', dpi=300)
    # plt.savefig('src/planning_node/test_results/cubic_polynomial_joint_data/joint_trajectory_visualization.png', dpi=300)
    
    # 显示图像
    plt.show()
    
    print("图像已保存为 'joint_trajectory_visualization.png'")

if __name__ == "__main__":
    # 指定CSV文件路径
    csv_file = "src/planning_node/test_results/nurbs_joint_data/visualization_data.csv"
    # csv_file = "src/planning_node/test_results/cubic_polynomial_joint_data/visualization_data.csv"
    
    # 绘制图像
    plot_joint_data(csv_file)