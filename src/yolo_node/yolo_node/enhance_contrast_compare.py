import cv2
import numpy as np
import matplotlib.pyplot as plt

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

        # 对比不同方法
        # 1. 原始图像
        # 2. 普通直方图均衡化
        # 3. CLAHE结果
        hist_eq = cv2.equalizeHist(img)

        # 显示结果
        plt.figure(figsize=(15, 10))

        # 显示原始图像
        plt.subplot(231)
        plt.imshow(img, cmap='gray')
        plt.title('origin')
        plt.axis('off')

        # 显示直方图均衡化结果
        plt.subplot(232)
        plt.imshow(hist_eq, cmap='gray')
        plt.title('normal histogram equalization')
        plt.axis('off')

        # 显示CLAHE结果
        plt.subplot(233)
        plt.imshow(enhanced_img, cmap='gray')
        plt.title('CLAHE enhancement')
        plt.axis('off')

        # 显示直方图
        plt.subplot(234)
        plt.hist(img.ravel(), 256, [0,256])
        plt.title('origin histogram')

        plt.subplot(235)
        plt.hist(hist_eq.ravel(), 256, [0,256])
        plt.title('normal histogram equalization')

        plt.subplot(236)
        plt.hist(enhanced_img.ravel(), 256, [0,256])
        plt.title('CLAHE enhancement histogram')

        plt.tight_layout()

        # 保存结果
        if save_path:
            plt.savefig(save_path)
            # 单独保存CLAHE结果
            cv2.imwrite(save_path.replace('.png', '_clahe.png'), enhanced_img)

        plt.show()
        return enhanced_img

    def batch_enhance(self, input_dir, output_dir):
        """
        批量处理文件夹中的图像
        Args:
            input_dir: 输入图像文件夹
            output_dir: 输出结果文件夹
        """
        import os
        os.makedirs(output_dir, exist_ok=True)

        for filename in os.listdir(input_dir):
            if filename.endswith(('.jpg', '.png', '.bmp')):
                input_path = os.path.join(input_dir, filename)
                output_path = os.path.join(output_dir, f'enhanced_{filename}')
                self.enhance_image(input_path, output_path)

    def adjust_parameters(self, image_path):
        """
        交互式调整CLAHE参数
        Args:
            image_path: 输入图像路径
        """
        def update(val):
            clip_limit = cv2.getTrackbarPos('Clip Limit', 'CLAHE Parameters')
            grid_size = cv2.getTrackbarPos('Grid Size', 'CLAHE Parameters')
            
            # 更新CLAHE参数
            clahe = cv2.createCLAHE(
                clipLimit=clip_limit/10.0,
                tileGridSize=(grid_size, grid_size)
            )
            enhanced = clahe.apply(img)
            cv2.imshow('CLAHE Parameters', enhanced)

        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        cv2.namedWindow('CLAHE Parameters')
        
        # 创建滑动条
        cv2.createTrackbar('Clip Limit', 'CLAHE Parameters', 20, 100, update)
        cv2.createTrackbar('Grid Size', 'CLAHE Parameters', 8, 16, update)
        
        update(0)  # 初始化显示
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main():
    # 使用示例
    enhancer = ContrastEnhancer(clip_limit=2.0, tile_grid_size=(8,8))
    
    # 1. 处理单张图像
    # image_path = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\11_20250310_183451.296.jpg'
    # enhanced_img = enhancer.enhance_image(
    #     image_path,
    #     save_path='enhanced_result.png'
    # )
    
    # 2. 批量处理
    input_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\images'
    output_folder = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\enhanced_images'
    enhancer.batch_enhance(input_folder, output_folder)
    
    # 3. 交互式调整参数
    # enhancer.adjust_parameters(image_path)

if __name__ == '__main__':
    main() 