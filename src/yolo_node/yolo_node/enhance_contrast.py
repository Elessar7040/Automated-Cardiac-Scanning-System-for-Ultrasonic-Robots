import cv2
import numpy as np
from tqdm import tqdm
import os

class ContrastEnhancer:
    def __init__(self, clip_limit=2.0, tile_grid_size=(8,8)):
        """
        初始化对比度增强器
        Args:
            clip_limit: CLAHE的对比度限制阈值
            tile_grid_size: 分块大小
        """
        self.clip_limit = clip_limit
        self.tile_grid_size = tile_grid_size
        self.clahe = cv2.createCLAHE(
            clipLimit=self.clip_limit,
            tileGridSize=self.tile_grid_size
        )

    def enhance_image(self, image_path, save_path=None):
        """
        使用CLAHE增强图像对比度
        Args:
            image_path: 输入图像路径
            save_path: 保存结果的路径（可选）
        Returns:
            enhanced_img: 增强后的图像
        """
        # 读取图像
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise ValueError("cannot read image")

        # 应用CLAHE
        enhanced_img = self.clahe.apply(img)

        # 保存结果
        if save_path:
            cv2.imwrite(save_path, enhanced_img)

        return enhanced_img

    def batch_enhance(self, input_dir, output_dir):
        """
        批量处理文件夹中的图像
        Args:
            input_dir: 输入图像文件夹
            output_dir: 输出结果文件夹
        """
        os.makedirs(output_dir, exist_ok=True)

        # 获取所有图片文件
        image_files = [f for f in os.listdir(input_dir) 
                      if f.endswith(('.jpg', '.png', '.bmp'))]
        
        # 使用tqdm显示进度
        for filename in tqdm(image_files, desc="增强处理进度"):
            input_path = os.path.join(input_dir, filename)
            output_path = os.path.join(output_dir, f'enhanced_{filename}')
            self.enhance_image(input_path, output_path)

def main():
    # 使用示例
    enhancer = ContrastEnhancer(clip_limit=2.0, tile_grid_size=(8,8))
    
    # 批量处理
    input_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\images'
    output_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\enhanced_images'
    enhancer.batch_enhance(input_folder, output_folder)

if __name__ == '__main__':
    main() 