import cv2
import numpy as np

def compare_videos(video1_path, video2_path):
    """
    并排显示两个视频文件
    Args:
        video1_path: 第一个视频的路径
        video2_path: 第二个视频的路径
    """
    # 打开视频文件
    cap1 = cv2.VideoCapture(video1_path)
    cap2 = cv2.VideoCapture(video2_path)
    
    # 检查视频是否成功打开
    if not cap1.isOpened() or not cap2.isOpened():
        print("错误：无法打开视频文件")
        return
    
    # 获取视频的基本信息
    width1 = int(cap1.get(cv2.CAP_PROP_FRAME_WIDTH))
    height1 = int(cap1.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps1 = cap1.get(cv2.CAP_PROP_FPS)
    
    width2 = int(cap2.get(cv2.CAP_PROP_FRAME_WIDTH))
    height2 = int(cap2.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps2 = cap2.get(cv2.CAP_PROP_FPS)
    
    print(f"视频1: {width1}x{height1} @ {fps1}fps")
    print(f"视频2: {width2}x{height2} @ {fps2}fps")
    
    # 创建窗口
    cv2.namedWindow('Video Comparison', cv2.WINDOW_NORMAL)
    
    # 播放控制变量
    paused = False
    
    while True:
        if not paused:
            # 读取两个视频的帧
            ret1, frame1 = cap1.read()
            ret2, frame2 = cap2.read()
            
            # 检查是否到达视频末尾
            if not ret1 or not ret2:
                print("视频播放完毕")
                break
            
            # 调整两个帧的大小使其高度相同
            target_height = min(height1, height2)
            aspect_ratio1 = width1 / height1
            aspect_ratio2 = width2 / height2
            
            target_width1 = int(target_height * aspect_ratio1)
            target_width2 = int(target_height * aspect_ratio2)
            
            frame1 = cv2.resize(frame1, (target_width1, target_height))
            frame2 = cv2.resize(frame2, (target_width2, target_height))
            
            # 在两个帧之间添加分隔线
            separator = np.zeros((target_height, 5, 3), dtype=np.uint8)
            separator[:, :] = [255, 255, 255]  # 白色分隔线
            
            # 添加标签
            cv2.putText(frame1, 'Video 1', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame2, 'Video 2', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 合并两个帧
            combined_frame = np.hstack((frame1, separator, frame2))
            
            # 显示合并后的帧
            cv2.imshow('Video Comparison', combined_frame)
        
        # 键盘控制
        key = cv2.waitKey(1) & 0xFF
        
        # 按ESC退出
        if key == 27:
            break
        # 按空格暂停/继续
        elif key == 32:
            paused = not paused
        # 按左箭头快退
        elif key == 81:  # 左箭头键
            current_pos = cap1.get(cv2.CAP_PROP_POS_FRAMES)
            cap1.set(cv2.CAP_PROP_POS_FRAMES, max(0, current_pos - 30))
            cap2.set(cv2.CAP_PROP_POS_FRAMES, max(0, current_pos - 30))
        # 按右箭头快进
        elif key == 83:  # 右箭头键
            current_pos = cap1.get(cv2.CAP_PROP_POS_FRAMES)
            cap1.set(cv2.CAP_PROP_POS_FRAMES, current_pos + 30)
            cap2.set(cv2.CAP_PROP_POS_FRAMES, current_pos + 30)
    
    # 释放资源
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # 设置视频路径
    # video1_path = r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/detect/predict14/Cardiac_15.avi'   # 替换为您的第一个视频路径
    # video2_path = r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/detect/predict18/Cardiac_15.avi'  # 替换为您的第二个视频路径
    
    # video1_path = r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/detect/predict14/Cardiac_15.avi'   # 替换为您的第一个视频路径
    # video2_path = r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/detect/predict19/Cardiac_15.avi'  # 替换为您的第二个视频路径
    

    video1_path = r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/detect/predict18/Cardiac_15.avi'   # 替换为您的第一个视频路径
    video2_path = r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/detect/predict19/Cardiac_15.avi'  # 替换为您的第二个视频路径
    
    # 开始比较
    compare_videos(video1_path, video2_path)