import cv2

# 焦距在x方向上的缩放因子
fx = 548.81623
# 焦距在x方向上的缩放因子
fy = 548.65548
# 主点在x方向上的坐标
cx = 316.76451
# 主点在y方向上的坐标
cy = 240.48396

# 像素坐标 (u, v) 和深度值 Z
u, v = 320, 240
Z = [v, u]  # 深度值

# 转换为相机坐标
X_cam = Z * (u - cx) / fx
Y_cam = Z * (v - cy) / fy
Z_cam = Z
print(f"Camera coordinates: ({X_cam}, {Y_cam}, {Z_cam})")
