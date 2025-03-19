from ultralytics import YOLO
import torch.multiprocessing as mp

def main():
    # Load a model
    # 不包含无标注图片
    # model = YOLO(r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp2/weights/best.pt')
    # 包含无标注图片
    # 100 epoch
    model = YOLO(r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp3/weights/best.pt')
    # 150 epoch
    # model = YOLO(r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp4/weights/best.pt')
    
    # Validate the model
    # metrics = model.val(data="data_labeled.yaml")
    metrics = model.val(data="data_unlabeled.yaml")
    
    # Print results
    print("\nValidation Results:")
    print(f"mAP50: {metrics.box.map50:.4f}")
    print(f"mAP50-95: {metrics.box.map:.4f}")
    print(f"metrics: {metrics}")
    print(f"Precision: {metrics.box.p:.4f}")
    print(f"Recall: {metrics.box.r:.4f}")

if __name__ == '__main__':
    # Add multiprocessing support
    mp.freeze_support()
    main()