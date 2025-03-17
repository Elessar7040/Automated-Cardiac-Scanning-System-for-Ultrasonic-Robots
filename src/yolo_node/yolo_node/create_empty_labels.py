import os
from pathlib import Path

def create_empty_labels():
    """为无标注图像创建空标签文件"""
    # 设置路径
    image_dir = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\unlabeled\images"
    label_dir = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\unlabeled\labels"
    
    # 创建标签目录
    os.makedirs(label_dir, exist_ok=True)
    
    # 支持的图像格式
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
    
    # 获取所有图像文件
    image_files = [f for f in os.listdir(image_dir) 
                  if any(f.lower().endswith(ext) for ext in image_extensions)]
    
    # 创建空标签文件
    count = 0
    for img_file in image_files:
        # 获取不带扩展名的文件名
        base_name = os.path.splitext(img_file)[0]
        # 创建对应的空txt文件
        label_path = os.path.join(label_dir, f"{base_name}.txt")
        
        # 创建空文件
        with open(label_path, 'w') as f:
            pass  # 创建空文件
        
        count += 1
    
    print(f"处理完成！")
    print(f"总共创建了 {count} 个空标签文件")
    print(f"标签文件保存在: {label_dir}")

if __name__ == '__main__':
    create_empty_labels()