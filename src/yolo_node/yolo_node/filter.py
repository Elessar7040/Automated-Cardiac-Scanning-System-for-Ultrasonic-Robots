import cv2
import numpy as np
from matplotlib import pyplot as plt

def apply_filters(image_path, save_path=None):
    """
    对超声图像应用多种滤波方法
    Args:
        image_path: 输入图像路径
        save_path: 保存结果的路径（可选）
    """
    # 读取图像
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("cannot read image")

    # 1. 高斯滤波
    gaussian = cv2.GaussianBlur(img, (5, 5), 0)
    
    # 2. 中值滤波（对椒盐噪声特别有效）
    median = cv2.medianBlur(img, 5)
    
    # 3. 双边滤波（保持边缘的同时降噪）
    bilateral = cv2.bilateralFilter(img, 9, 75, 75)
    
    # 4. 非局部均值去噪（NLMeans）
    nlm = cv2.fastNlMeansDenoising(img, None, 10, 7, 21)

    # 显示结果
    plt.figure(figsize=(15, 10))
    
    plt.subplot(231), plt.imshow(img, cmap='gray')
    plt.title('origin'), plt.axis('off')
    
    plt.subplot(232), plt.imshow(gaussian, cmap='gray')
    plt.title('gaussian'), plt.axis('off')
    
    plt.subplot(233), plt.imshow(median, cmap='gray')
    plt.title('median'), plt.axis('off')
    
    plt.subplot(234), plt.imshow(bilateral, cmap='gray')
    plt.title('bilateral'), plt.axis('off')
    
    plt.subplot(235), plt.imshow(nlm, cmap='gray')
    plt.title('nlm'), plt.axis('off')

    plt.tight_layout()
    
    # 保存结果
    if save_path:
        plt.savefig(save_path)
        
        # 也可以单独保存每种滤波结果
        cv2.imwrite(save_path.replace('.png', '_gaussian.png'), gaussian)
        cv2.imwrite(save_path.replace('.png', '_median.png'), median)
        cv2.imwrite(save_path.replace('.png', '_bilateral.png'), bilateral)
        cv2.imwrite(save_path.replace('.png', '_nlm.png'), nlm)
    
    plt.show()

def adaptive_filter(image_path, method='gaussian', save_path=None):
    """
    应用单个指定的滤波方法
    Args:
        image_path: 输入图像路径
        method: 滤波方法 ('gaussian', 'median', 'bilateral', 'nlm')
        save_path: 保存结果的路径（可选）
    """
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError("cannot read image")

    # 根据方法选择滤波器
    if method == 'gaussian':
        filtered = cv2.GaussianBlur(img, (5, 5), 0)
    elif method == 'median':
        filtered = cv2.medianBlur(img, 15)
    elif method == 'bilateral':
        filtered = cv2.bilateralFilter(img, 9, 75, 75)
    elif method == 'nlm':
        filtered = cv2.fastNlMeansDenoising(img, None, 10, 7, 21)
    else:
        raise ValueError("unsupported filter method")

    # 显示结果
    plt.figure(figsize=(10, 5))
    
    plt.subplot(121), plt.imshow(img, cmap='gray')
    plt.title('origin'), plt.axis('off')
    
    plt.subplot(122), plt.imshow(filtered, cmap='gray')
    plt.title(f'{method} filter result'), plt.axis('off')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path)
    
    plt.show()
    
    return filtered

if __name__ == '__main__':
    # 使用示例
    image_path = r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\11_20250310_183451.296.jpg'
    
    # 1. 显示所有滤波方法的结果
    apply_filters(image_path, 'filter_result.png')
    
    # # 2. 使用单个指定的滤波方法
    # filtered_image = adaptive_filter(
    #     image_path,
    #     method='nlm',  # 可选: 'gaussian', 'median', 'bilateral', 'nlm'
    #     save_path='single_filter_result.png'
    # ) 