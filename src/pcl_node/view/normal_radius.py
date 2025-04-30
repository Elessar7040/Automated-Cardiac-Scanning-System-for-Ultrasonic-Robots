#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_normal_radius_benchmark(csv_file, output_dir=None):
    """
    读取法向量搜索半径基准测试结果数据并绘制图表
    
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
    
    # 创建图表
    try:
        # 将DataFrame列转换为numpy数组
        radius_values = data['搜索半径'].to_numpy()
        normal_counts = data['法向量数量'].to_numpy()
        process_times = data['处理时间(ms)'].to_numpy()
        cosine_similarities = data['平均余弦相似度'].to_numpy()
        angle_differences = data['平均角度差异(度)'].to_numpy()
        consistency_rates = data['法向量一致率(%)'].to_numpy()
        
        # 创建四个子图
        fig, axes = plt.subplots(2, 2, figsize=(16, 14))
        
        # 第一个子图：法向量数量随搜索半径的变化
        axes[0, 0].plot(radius_values, normal_counts, 'bo-', linewidth=2)
        axes[0, 0].set_title('Trend of Normal Count', fontsize=16)
        axes[0, 0].set_xlabel('Search Radius', fontsize=14)
        axes[0, 0].set_ylabel('Normal Count', fontsize=14)
        axes[0, 0].grid(True)
        
        # 第二个子图：处理时间随搜索半径的变化
        axes[0, 1].plot(radius_values, process_times, 'ro-', linewidth=2)
        axes[0, 1].set_title('Trend of Processing Time', fontsize=16)
        axes[0, 1].set_xlabel('Search Radius', fontsize=14)
        axes[0, 1].set_ylabel('Processing Time (ms)', fontsize=14)
        axes[0, 1].grid(True)
        
        # 第三个子图：精度指标随搜索半径的变化
        ax3 = axes[1, 0]
        ax3.plot(radius_values, cosine_similarities, 'go-', linewidth=2, label='Cosine Similarity')
        ax3.set_title('Trend of Cosine Similarity', fontsize=16)
        ax3.set_xlabel('Search Radius', fontsize=14)
        ax3.set_ylabel('Average Cosine Similarity', fontsize=14)
        ax3.grid(True)
        ax3.set_ylim([0, 1.05])  # 余弦相似度范围是0-1
        
        # 第四个子图：角度差异和一致率
        ax4 = axes[1, 1]
        line1 = ax4.plot(radius_values, angle_differences, 'mo-', linewidth=2, label='Angle Difference')
        ax4.set_xlabel('Search Radius', fontsize=14)
        ax4.set_ylabel('Average Angle Difference (degrees/°)', fontsize=14)
        ax4.tick_params(axis='y', labelcolor='m')
        
        # 创建一个共享x轴的新y轴，用于绘制一致率
        ax4_twin = ax4.twinx()
        line2 = ax4_twin.plot(radius_values, consistency_rates, 'c^-', linewidth=2, label='Normal Consistency Rate (%)')
        ax4_twin.set_ylabel('Normal Consistency Rate (%)', color='c', fontsize=14)
        ax4_twin.tick_params(axis='y', labelcolor='c')
        
        # 添加标题和图例
        ax4.set_title('Trend of Angle Difference and Normal Consistency Rate', fontsize=16)
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax4.legend(lines, labels, loc='upper right')
        
        ax4.grid(True)
        
        plt.tight_layout()
        
        # 保存综合图表
        chart_path = os.path.join(output_dir, f"{base_name}_all_chart.png")
        plt.savefig(chart_path, dpi=300)
        print(f"综合图表已保存到: {chart_path}")
        
        plt.close()
        
        # 创建单独的图表
        # 法向量数量图表
        plt.figure(figsize=(12, 6))
        plt.plot(radius_values, normal_counts, 'bo-', linewidth=2, markersize=8)
        plt.title('Trend of Normal Count', fontsize=18)
        plt.xlabel('Search Radius', fontsize=16)
        plt.ylabel('Normal Count', fontsize=16)
        plt.grid(True)
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        
        counts_chart_path = os.path.join(output_dir, f"{base_name}_counts_chart.png")
        plt.savefig(counts_chart_path, dpi=300)
        print(f"法向量数量图表已保存到: {counts_chart_path}")
        plt.close()
        
        # 处理时间图表
        plt.figure(figsize=(12, 6))
        plt.plot(radius_values, process_times, 'ro-', linewidth=2, markersize=8)
        plt.title('Trend of Processing Time', fontsize=18)
        plt.xlabel('Search Radius', fontsize=16)
        plt.ylabel('Processing Time (ms)', fontsize=16)
        plt.grid(True)
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        
        time_chart_path = os.path.join(output_dir, f"{base_name}_time_chart.png")
        plt.savefig(time_chart_path, dpi=300)
        print(f"处理时间图表已保存到: {time_chart_path}")
        plt.close()
        
        # 余弦相似度图表
        plt.figure(figsize=(12, 6))
        plt.plot(radius_values, cosine_similarities, 'go-', linewidth=2, markersize=8)
        plt.title('Trend of Cosine Similarity', fontsize=18)
        plt.xlabel('Search Radius', fontsize=16)
        plt.ylabel('Average Cosine Similarity', fontsize=16)
        plt.grid(True)
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        plt.ylim([0, 1.05])
        
        cosine_chart_path = os.path.join(output_dir, f"{base_name}_cosine_chart.png")
        plt.savefig(cosine_chart_path, dpi=300)
        print(f"余弦相似度图表已保存到: {cosine_chart_path}")
        plt.close()
        
        # 角度差异图表
        plt.figure(figsize=(12, 6))
        plt.plot(radius_values, angle_differences, 'mo-', linewidth=2, markersize=8)
        plt.title('Trend of Angle Difference', fontsize=18)
        plt.xlabel('Search Radius', fontsize=16)
        plt.ylabel('Average Angle Difference (degrees/°)', fontsize=16)
        plt.grid(True)
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        
        angle_chart_path = os.path.join(output_dir, f"{base_name}_angle_chart.png")
        plt.savefig(angle_chart_path, dpi=300)
        print(f"角度差异图表已保存到: {angle_chart_path}")
        plt.close()
        
        # 一致率图表
        plt.figure(figsize=(12, 6))
        plt.plot(radius_values, consistency_rates, 'c^-', linewidth=2, markersize=8)
        plt.title('Trend of Normal Consistency Rate', fontsize=18)
        plt.xlabel('Search Radius', fontsize=16)
        plt.ylabel('Normal Consistency Rate (%)', fontsize=16)
        plt.grid(True)
        plt.xticks(fontsize=14)
        plt.yticks(fontsize=14)
        plt.ylim([0, 105])
        
        consistency_chart_path = os.path.join(output_dir, f"{base_name}_consistency_chart.png")
        plt.savefig(consistency_chart_path, dpi=300)
        print(f"一致率图表已保存到: {consistency_chart_path}")
        plt.close()
        
        # 创建处理时间与精度的权衡图
        plt.figure(figsize=(12, 7))
        
        # 主y轴：处理时间
        fig, ax1 = plt.subplots(figsize=(12, 7))
        line1 = ax1.plot(radius_values, process_times, 'ro-', linewidth=2, markersize=8, label='Processing Time')
        ax1.set_xlabel('Search Radius', fontsize=16)
        ax1.set_ylabel('Processing Time (ms)', color='r', fontsize=16)
        ax1.tick_params(axis='y', labelcolor='r')
        
        # 第二个y轴：余弦相似度
        ax2 = ax1.twinx()
        line2 = ax2.plot(radius_values, cosine_similarities, 'go-', linewidth=2, markersize=8, label='Cosine Similarity')
        ax2.set_ylabel('Average Cosine Similarity', color='g', fontsize=16)
        ax2.tick_params(axis='y', labelcolor='g')
        ax2.set_ylim([0, 1.05])
        
        # 第三个y轴：一致率
        ax3 = ax1.twinx()
        # 调整第三个y轴的位置
        ax3.spines['right'].set_position(('outward', 60))
        line3 = ax3.plot(radius_values, consistency_rates, 'c^-', linewidth=2, markersize=8, label='Normal Consistency Rate')
        ax3.set_ylabel('Normal Consistency Rate (%)', color='c', fontsize=16)
        ax3.tick_params(axis='y', labelcolor='c')
        ax3.set_ylim([0, 105])
        
        # 标题和图例
        plt.title('Tradeoff between Processing Time and Accuracy', fontsize=18)
        plt.grid(True)
        
        # 组合所有线条添加图例
        lines = line1 + line2 + line3
        labels = [l.get_label() for l in lines]
        plt.legend(lines, labels, loc='upper center', bbox_to_anchor=(0.5, -0.1), 
                 fancybox=True, shadow=True, ncol=3, fontsize=14)
        
        plt.tight_layout()
        
        tradeoff_chart_path = os.path.join(output_dir, f"{base_name}_tradeoff_chart.png")
        plt.savefig(tradeoff_chart_path, dpi=300, bbox_inches='tight')
        print(f"权衡图表已保存到: {tradeoff_chart_path}")
        plt.close()
        
        return True
        
    except Exception as e:
        print(f"创建图表时出错: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    csv_file = "/home/elessar/russ_ws/ws7/src/pcl_node/test_results/normal_radius_benchmark_results.csv"
    output = "/home/elessar/russ_ws/ws7/src/pcl_node/test_results"
    
    success = plot_normal_radius_benchmark(csv_file, output)
    if success:
        print("图表生成成功!")
    else:
        print("图表生成失败!")