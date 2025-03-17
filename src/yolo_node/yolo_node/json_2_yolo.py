import json
import os
import numpy as np
from pathlib import Path
import cv2

def read_json_sample(json_path):
    """读取一个JSON文件并打印其结构"""
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    print("JSON文件结构示例：")
    print(json.dumps(data, indent=2, ensure_ascii=False))

def get_rotated_box_points(cx, cy, w, h, angle_deg):
    """
    获取旋转框的四个顶点坐标
    Args:
        cx, cy: 中心点坐标
        w, h: 宽度和高度
        angle_deg: 角度（度）
    Returns:
        numpy array: 四个顶点坐标 (4,2)
    """
    angle_rad = np.deg2rad(angle_deg)
    half_w, half_h = w/2, h/2
    
    # 生成未旋转的矩形框四个顶点（相对于中心点）
    points = np.array([
        [-half_w, -half_h],
        [half_w, -half_h],
        [half_w, half_h],
        [-half_w, half_h]
    ])
    
    # 旋转矩阵
    rotation_matrix = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad), np.cos(angle_rad)]
    ])
    
    # 旋转点并平移到中心位置
    rotated_points = np.dot(points, rotation_matrix.T) + [cx, cy]
    return rotated_points

def polygon_to_yolo_obb(points, img_width, img_height):
    """
    将多边形点转换为YOLO-OBB格式的四个顶点
    Returns:
        list: 归一化后的四个顶点坐标 [x1,y1,x2,y2,x3,y3,x4,y4]
    """
    # 获取最小外接矩形
    rect = cv2.minAreaRect(points.astype(np.float32))
    center, size, angle = rect
    
    # 获取旋转框的四个顶点
    box_points = get_rotated_box_points(center[0], center[1], size[0], size[1], angle)
    
    # 归一化坐标
    box_points[:, 0] /= img_width
    box_points[:, 1] /= img_height
    
    # 转换为一维数组 [x1,y1,x2,y2,x3,y3,x4,y4]
    return box_points.flatten()

def convert_labelme_to_yolo_obb(json_path, img_width, img_height, class_map):
    """将单个JSON文件转换为YOLO-OBB格式"""
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    yolo_lines = []
    for shape in data['shapes']:
        # 获取类别索引
        label = shape['label']
        if label not in class_map:
            print(f"警告：未知类别 {label} 在文件 {json_path}")
            continue
        class_id = class_map[label]
        
        # 获取多边形点
        points = np.array(shape['points'])
        
        # 转换为YOLO-OBB格式的四个顶点
        box_points = polygon_to_yolo_obb(points, img_width, img_height)
        
        # 格式化为YOLO-OBB格式：class_id x1 y1 x2 y2 x3 y3 x4 y4
        yolo_line = f"{class_id} " + " ".join([f"{coord:.6f}" for coord in box_points])
        yolo_lines.append(yolo_line)
    
    return yolo_lines

def convert_dataset(json_dir, output_dir, class_names):
    """转换整个数据集"""
    os.makedirs(output_dir, exist_ok=True)
    class_map = {name: i for i, name in enumerate(class_names)}
    
    json_files = list(Path(json_dir).glob('*.json'))
    for json_path in json_files:
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            img_width = data['imageWidth']
            img_height = data['imageHeight']
        
        yolo_lines = convert_labelme_to_yolo_obb(json_path, img_width, img_height, class_map)
        
        output_path = os.path.join(output_dir, json_path.stem + '.txt')
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(yolo_lines))
        print(f"已处理: {json_path.name}")

if __name__ == '__main__':
    # 首先读取一个样本JSON文件来查看格式
    json_dir = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets\train\labels_json"
    sample_json = next(Path(json_dir).glob('*.json'))
    read_json_sample(sample_json)
    
    # 设置类别名称
    class_names = ['apical_four_chamber']  # 根据您的实际类别修改
    
    # 转换训练集、验证集和测试集
    dataset_root = r"E:\YOLO\YOLO11\Projects\cardiac_ultrasound\datasets"
    for split in ['train', 'val', 'test']:
        json_dir = os.path.join(dataset_root, split, 'labels_json')
        output_dir = os.path.join(dataset_root, split, 'labels_yolo')
        print(f"\n转换{split}集...")
        convert_dataset(json_dir, output_dir, class_names)
        print(f"{split}集转换完成！")