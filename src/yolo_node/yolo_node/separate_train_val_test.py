import os
import random
import shutil
from pathlib import Path

def split_dataset(image_dir, label_dir, output_dir, train_ratio=0.8, val_ratio=0.1):
    """
    划分数据集为train, val, test
    
    Args:
        image_dir: 图片目录
        label_dir: 标签目录
        output_dir: 输出目录
        train_ratio: 训练集比例
        val_ratio: 验证集比例
    """
    # 创建输出目录结构
    for split in ['train', 'val', 'test']:
        os.makedirs(os.path.join(output_dir, split, 'images'), exist_ok=True)
        os.makedirs(os.path.join(output_dir, split, 'labels'), exist_ok=True)

    # 获取所有图片文件
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
    images = [f for f in os.listdir(image_dir) 
             if any(f.lower().endswith(ext) for ext in image_extensions)]
    
    # 随机打乱数据
    random.seed(42)  # 设置随机种子，确保结果可复现
    random.shuffle(images)
    
    # 计算划分点
    total = len(images)
    train_end = int(total * train_ratio)
    val_end = int(total * (train_ratio + val_ratio))
    
    # 划分数据集
    splits = {
        'train': images[:train_end],
        'val': images[train_end:val_end],
        'test': images[val_end:]
    }
    
    # 复制文件到对应目录
    for split, img_list in splits.items():
        for img in img_list:
            # 复制图片
            src_img = os.path.join(image_dir, img)
            dst_img = os.path.join(output_dir, split, 'images', img)
            shutil.copy2(src_img, dst_img)
            
            # 复制对应的标签文件
            # label_name = os.path.splitext(img)[0] + '.json'
            label_name = os.path.splitext(img)[0] + '.txt'
            src_label = os.path.join(label_dir, label_name)
            dst_label = os.path.join(output_dir, split, 'labels', label_name)
            if os.path.exists(src_label):
                shutil.copy2(src_label, dst_label)
    
    # 打印统计信息
    print(f"数据集划分完成！")
    print(f"总数据量: {total}")
    print(f"训练集: {len(splits['train'])} 张图片 ({train_ratio*100:.0f}%)")
    print(f"验证集: {len(splits['val'])} 张图片 ({val_ratio*100:.0f}%)")
    print(f"测试集: {len(splits['test'])} 张图片 ({(1-train_ratio-val_ratio)*100:.0f}%)")

if __name__ == '__main__':
    # 您的具体路径
    # image_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\enhanced_labels\labeled\images"
    # label_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\ultrasound_pictures\enhanced_labels\labeled\labels"
    # output_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets"  # 这里您可以修改为想要的输出路径
    
    image_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\with_unlabeled\total\images"
    label_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\with_unlabeled\total\labels"
    output_directory = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\with_unlabeled"  # 这里您可以修改为想要的输出路径

    split_dataset(image_directory, label_directory, output_directory)