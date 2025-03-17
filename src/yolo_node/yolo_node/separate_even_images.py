import os
import shutil
from tqdm import tqdm

def separate_even_images(input_dir, output_dir):
    """
    将输入文件夹中偶数编号的图片移动到输出文件夹
    Args:
        input_dir: 输入图像文件夹路径
        output_dir: 输出图像文件夹路径（偶数编号图片）
    """
    # 创建输出文件夹
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取所有图片文件
    image_files = [f for f in os.listdir(input_dir) 
                   if f.endswith(('.jpg', '.png', '.bmp'))]
    
    # 按文件名排序
    image_files.sort()
    
    print(f"找到 {len(image_files)} 张图片")
    
    # 移动偶数编号的图片
    moved_count = 0
    for filename in tqdm(image_files, desc="处理进度"):
        try:
            # 获取文件编号（假设文件名格式为 "001.jpg" 这样的格式）
            file_number = int(filename.split('.')[0])
            
            # 如果是偶数编号
            if file_number % 2 == 0:
                # 源文件和目标文件的完整路径
                src_path = os.path.join(input_dir, filename)
                dst_path = os.path.join(output_dir, filename)
                
                # 移动文件
                shutil.move(src_path, dst_path)
                # 复制文件
                # shutil.copy(src_path, dst_path)
                
                moved_count += 1
                
        except ValueError:
            print(f"警告: 无法从文件名 '{filename}' 中获取编号")
            continue
    
    print(f"\n处理完成！")
    print(f"共移动了 {moved_count} 张偶数编号的图片到 {output_dir}")

def main():
    # 指定输入和输出文件夹路径
    input_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\images'
    output_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\even_images'
    
    try:
        separate_even_images(input_folder, output_folder)
    except Exception as e:
        print(f"发生错误: {str(e)}")

if __name__ == '__main__':
    main() 