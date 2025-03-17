import warnings
warnings.filterwarnings('ignore')
from ultralytics import YOLO


if __name__ == '__main__':
    # 开始训练
    model = YOLO(r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\models\ultralytics\yolo11n-obb.pt')
    # model = YOLO(model=r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\ultralytics\cfg\models\11\yolo11-obb.yaml')
    # model.load(r'E:\YOLO\YOLO11\Projects\cardiac_ultrasound\models\ultralytics\yolo11n-obb.pt')  # 加载预训练权重,改进或者做对比实验时候不建议打开，因为用预训练模型整体精度没有很明显的提升
    model.train(
        # data=r'data_labeled.yaml',
        data=r'data_unlabeled.yaml',
        imgsz=640,
        epochs=150,        # 增加训练轮数
        batch=32,         # 根据您的GPU显存调整
        workers=8,
        device=0,
        optimizer='SGD',
        close_mosaic=10,
        resume=False,
        project='runs/train',
        name='exp',
        single_cls=False,
        cache=False,
        
        # 数据增强参数
        augment=True,     
        degrees=15,       # 旋转角度范围
        translate=0.2,    # 平移范围
        scale=0.3,        # 缩放范围
        fliplr=0.5,       # 水平翻转概率
        mosaic=0.7,       # mosaic增强概率
        mixup=0.2,        # mixup增强概率
        
        # 样本权重
        # pos_weight=1.0,   # 正样本权重
        # neg_weight=0.5,   # 负样本权重
        
        # 学习率设置
        # lr0=0.01,        # 初始学习率
        # lrf=0.01,        # 最终学习率
        # momentum=0.937,   # SGD动量
        # weight_decay=0.0005,  # 权重衰减
        
        # 保存设置
        # save_period=10,   # 每10轮保存一次
        save=True,        # 保存最后一轮
    )
