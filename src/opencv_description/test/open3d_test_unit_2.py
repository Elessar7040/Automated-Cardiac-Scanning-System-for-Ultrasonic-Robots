import open3d as o3d 
import numpy as np 
print("->正在加载点云... ") 
pcd = o3d.io.read_point_cloud("src/opencv_description/pointcloud_output/output.pcd") 
print(pcd) 
print("->正在估计法线并可视化...") 
radius = 0.5 # 搜索半径 
max_nn = 10 # 邻域内用于估算法线的最大点数 
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius, max_nn)) # 执行法线估计 
# o3d.visualization.draw_geometries([pcd], point_show_normal=True) 
print("->正在打印前10个点的法向量...") 
print(np.asarray(pcd.normals)[:10, :])
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

# -3.72333503e-06 -1.00000000e+00 -3.09755159e-05
# target_point = np.array([1.0, 2.0, 3.0])
target_point = np.array([0.8334480561426556, 1.7222589845005871, 0.4072678800794594])
# 查询最近的点索引
[_, idx, _] = pcd_tree.search_knn_vector_3d(target_point, 1)
nearest_idx = idx[0]

# Step 3: 获取最近点的法向量
normal = pcd.normals[nearest_idx]

print(f"Target point: {target_point}")
print(f"Nearest point index: {nearest_idx}")
print(f"Normal at nearest point: {normal}")
print(f"Type of normal: {type(normal)}")
# print(np.asarray(pcd.normals))
