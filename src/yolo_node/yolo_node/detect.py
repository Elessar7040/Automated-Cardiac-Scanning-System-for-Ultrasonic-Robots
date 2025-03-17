from ultralytics import YOLO

if __name__ == '__main__':

    # Load a model
    # 不包含无标注图片
    # model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp2/weights/best.pt')
    # 包含无标注图片
    # model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp3/weights/best.pt')
    model = YOLO(model=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/train/exp4/weights/best.pt')
    
    model.predict(source=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/Cardiac_15.avi',
                  save=True,
                  show=True,
                  project=r'/home/elessar/russ_ws/ws7/src/yolo_node/yolo_node/runs/detect',
                  # device='cpu',  # 指定使用CPU推理
                  )

