#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_voxel_benchmark(csv_file, output_dir=None):
    """
    读取体素基准测试结果数据并绘制图表
    
    参数:
        csv_file: CSV文件路径
        output_dir: 输出目录，如果为None则使用CSV文件所在目录
    """
    # 检查文件是否存在
    if not os.path.exists(csv_file):
        print(f"错误: 找不到文件 {csv_file}")
        return False
        
    # 读取CSV文件
    try:
        data = pd.read_csv(csv_file)
        print(f"成功读取数据: {len(data)} 行")
    except Exception as e:
        print(f"读取CSV文件时出错: {e}")
        return False
        
    # 确定输出目录
    if output_dir is None:
        output_dir = os.path.dirname(csv_file)
    os.makedirs(output_dir, exist_ok=True)
    
    # 设置图表输出文件名
    base_name = os.path.splitext(os.path.basename(csv_file))[0]
    chart_path = os.path.join(output_dir, f"{base_name}_chart.png")
    
    # 创建图表
    try:
        # 将DataFrame列转换为numpy数组
        voxel_sizes = data['体素大小'].to_numpy()
        point_counts = data['点云数量'].to_numpy()
        process_times = data['处理时间(ms)'].to_numpy()
        
        # 创建两个子图
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))
        
        # 第一个子图：点云数量随体素大小的变化
        ax1.plot(voxel_sizes, point_counts, 'bo-', linewidth=2)
        ax1.set_title('Trend of Point Cloud Count', fontsize=16)
        ax1.set_xlabel('Voxel Size', fontsize=14)
        ax1.set_ylabel('Point Cloud Count', fontsize=14)
        ax1.grid(True)
        
        # 在数据点上标出值
        for i, txt in enumerate(point_counts):
            ax1.annotate(str(txt), (voxel_sizes[i], point_counts[i]), 
                         xytext=(5, 5), textcoords='offset points')
        
        # 第二个子图：处理时间随体素大小的变化
        ax2.plot(voxel_sizes, process_times, 'ro-', linewidth=2)
        ax2.set_title('Trend of Processing Time', fontsize=16)
        ax2.set_xlabel('Voxel Size', fontsize=14)
        ax2.set_ylabel('Processing Time (ms)', fontsize=14)
        ax2.grid(True)
        
        # 在数据点上标出值
        for i, txt in enumerate(process_times):
            ax2.annotate(str(txt), (voxel_sizes[i], process_times[i]), 
                         xytext=(5, 5), textcoords='offset points')
        
        plt.tight_layout()
        
        # 保存图表
        plt.savefig(chart_path, dpi=300)
        print(f"图表已保存到: {chart_path}")
        
        # 额外创建一个单点云数量图表
        plt.figure(figsize=(12, 6))
        plt.plot(voxel_sizes, point_counts, 'bo-', linewidth=2, markersize=8)
        plt.title('Trend of Point Cloud Count', fontsize=18)
        plt.xlabel('Voxel Size', fontsize=16)
        plt.ylabel('Point Cloud Count', fontsize=16)
        plt.grid(True)
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        
        # 在数据点上标出值
        for i, txt in enumerate(point_counts):
            plt.annotate(str(txt), (voxel_sizes[i], point_counts[i]), 
                        xytext=(0, 10), textcoords='offset points', 
                        ha='center', fontsize=12)
        
        points_chart_path = os.path.join(output_dir, f"{base_name}_points_chart.png")
        plt.savefig(points_chart_path, dpi=300)
        print(f"点云数量图表已保存到: {points_chart_path}")
        
        # 额外创建一个单处理时间图表
        plt.figure(figsize=(12, 6))
        plt.plot(voxel_sizes, process_times, 'ro-', linewidth=2, markersize=8)
        plt.title('Trend of Processing Time', fontsize=18)
        plt.xlabel('Voxel Size', fontsize=16)
        plt.ylabel('Processing Time (ms)', fontsize=16)
        plt.grid(True)
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        
        # 在数据点上标出值
        for i, txt in enumerate(process_times):
            plt.annotate(str(txt), (voxel_sizes[i], process_times[i]), 
                        xytext=(0, 10), textcoords='offset points', 
                        ha='center', fontsize=12)
        
        time_chart_path = os.path.join(output_dir, f"{base_name}_time_chart.png")
        plt.savefig(time_chart_path, dpi=300)
        print(f"处理时间图表已保存到: {time_chart_path}")
        
        return True
        
    except Exception as e:
        print(f"创建图表时出错: {e}")
        return False

if __name__ == "__main__":
    csv_file = "/home/elessar/russ_ws/ws7/src/pcl_node/test_results/voxel_benchmark_results.csv"
    output = "/home/elessar/russ_ws/ws7/src/pcl_node/test_results"
    
    success = plot_voxel_benchmark(csv_file, output)
    if success:
        print("图表生成成功!")
    else:
        print("图表生成失败!")