import os
import shutil
from pathlib import Path

def organize_dataset(source_dir, output_dir):
    """
    整理数据集，将有标注和无标注的图片分开
    
    Args:
        source_dir: 源文件夹路径，包含图片和JSON文件
        output_dir: 输出文件夹路径
    """
    # 创建必要的文件夹
    labeled_dir = os.path.join(output_dir, 'labeled')
    unlabeled_dir = os.path.join(output_dir, 'unlabeled')
    
    os.makedirs(labeled_dir, exist_ok=True)
    os.makedirs(unlabeled_dir, exist_ok=True)
    
    # 获取所有图片文件
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
    json_files = set(f.stem for f in Path(source_dir).glob('*.json'))
    
    # 遍历源文件夹
    for file in os.listdir(source_dir):
        file_path = os.path.join(source_dir, file)
        file_name = os.path.splitext(file)[0]
        file_ext = os.path.splitext(file)[1].lower()
        
        # 处理图片文件
        if file_ext in image_extensions:
            # 判断是否有对应的JSON文件
            if file_name in json_files:
                # 有标注的图片
                shutil.copy2(file_path, os.path.join(labeled_dir, file))
                # 复制对应的JSON文件
                json_file = file_name + '.json'
                json_path = os.path.join(source_dir, json_file)
                if os.path.exists(json_path):
                    shutil.copy2(json_path, os.path.join(labeled_dir, json_file))
            else:
                # 无标注的图片
                shutil.copy2(file_path, os.path.join(unlabeled_dir, file))

    # 统计文件数量
    labeled_count = len([f for f in os.listdir(labeled_dir) if any(f.lower().endswith(ext) for ext in image_extensions)])
    unlabeled_count = len([f for f in os.listdir(unlabeled_dir) if any(f.lower().endswith(ext) for ext in image_extensions)])
    
    print(f"处理完成！")
    print(f"有标注图片数量: {labeled_count}")
    print(f"无标注图片数量: {unlabeled_count}")

# 使用示例
if __name__ == '__main__':
    source_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\enhanced_images"  # 请替换为您的源文件夹路径
    output_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\enhanced_labels"      # 请替换为您想要输出的文件夹路径
    
    organize_dataset(source_directory, output_directory)