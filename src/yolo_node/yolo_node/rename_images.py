import os
from tqdm import tqdm

def rename_images(input_dir):
    """
    将文件夹中的jpg图片按顺序重命名为001-999
    Args:
        input_dir: 输入图像文件夹路径
    """
    # 获取所有jpg图片
    image_files = [f for f in os.listdir(input_dir) 
                   if f.lower().endswith('.jpg')]
    
    # 按文件名排序
    image_files.sort()
    
    # 检查文件数量
    if len(image_files) > 999:
        raise ValueError(f"图片数量({len(image_files)})超过999，无法重命名")
    
    print(f"找到 {len(image_files)} 张图片")
    
    # 重命名文件
    for idx, old_name in enumerate(tqdm(image_files, desc="重命名进度")):
        # 生成新文件名 (001-999)
        new_name = f"{idx + 1:03d}.jpg"
        
        # 完整的文件路径
        old_path = os.path.join(input_dir, old_name)
        new_path = os.path.join(input_dir, new_name)
        
        # 如果新文件名已存在，先创建临时文件名
        if os.path.exists(new_path):
            temp_path = os.path.join(input_dir, f"temp_{new_name}")
            os.rename(old_path, temp_path)
            old_path = temp_path
            
        # 重命名文件
        os.rename(old_path, new_path)

def main():
    # 指定图片文件夹路径
    # input_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\images'
    input_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\enhanced_images'
    
    try:
        rename_images(input_folder)
        print("重命名完成！")
    except Exception as e:
        print(f"发生错误: {str(e)}")

if __name__ == '__main__':
    main() 