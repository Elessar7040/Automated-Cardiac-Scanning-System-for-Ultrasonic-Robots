import os
import psutil
import torch
import multiprocessing

def check_system_resources():
    """检查系统资源和推荐的worker数量"""
    
    # CPU信息
    cpu_count = os.cpu_count()  # 逻辑CPU核心数
    physical_cores = psutil.cpu_count(logical=False)  # 物理CPU核心数
    
    # 内存信息
    memory = psutil.virtual_memory()
    total_memory_gb = memory.total / (1024 ** 3)
    available_memory_gb = memory.available / (1024 ** 3)
    
    # CUDA信息
    cuda_available = torch.cuda.is_available()
    if cuda_available:
        gpu_count = torch.cuda.device_count()
        gpu_name = torch.cuda.get_device_name(0)
    
    # 打印系统信息
    print("系统资源信息：")
    print(f"CPU物理核心数: {physical_cores}")
    print(f"CPU逻辑核心数: {cpu_count}")
    print(f"总内存: {total_memory_gb:.1f}GB")
    print(f"可用内存: {available_memory_gb:.1f}GB")
    print(f"CUDA是否可用: {cuda_available}")
    if cuda_available:
        print(f"GPU数量: {gpu_count}")
        print(f"GPU型号: {gpu_name}")
    
    # 推荐的worker数量
    recommended_workers = min(
        cpu_count,  # CPU核心数
        int(available_memory_gb / 2),  # 基于可用内存估算
        16  # 一般建议的最大值
    )
    
    print(f"\n推荐的worker数量: {recommended_workers}")
    print("\n建议：")
    print(f"1. 保守设置: {recommended_workers // 2}")
    print(f"2. 平衡设置: {recommended_workers}")
    print(f"3. 激进设置: {min(recommended_workers * 2, cpu_count)}")
    
    return recommended_workers

if __name__ == '__main__':
    workers = check_system_resources()