import copy # 点云深拷贝 
import open3d as o3d 
import numpy as np 
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header



import transforms3d as tfs
import numpy as np 

# Rotation: x=0.005273056135837967, y=0.0022109277785422312, z=0.38665542379727236, w=0.9222065332264232
# Rotation: x=5.832428681889309e-05, y=3.2719411675896785e-05, z=0.4892708764756398, w=0.8721318736065532
# Translation: x=0.797089454454915, y=-0.26251446140768153, z=0.17284695975835598
# 四元数转旋转矩阵
rotate_matrix = tfs.quaternions.quat2mat([0.707,0.00,0.00,0.707])
print("rotate_matrix: \n", rotate_matrix)
# # 旋转矩阵转四元数
# quat = tfs.quaternions.mat2quat(np.asarray([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]]))
# print("quat:", quat)
# euler = tfs.euler.quat2euler([0.8721318736065532,0.00,0.00,0.4892708764756398],"sxyz")
# print("Euler:", euler)

T_base2ee = np.eye(4)
T_base2ee[:3, :3] = rotate_matrix
# 0.81692; 0.19139; 0.1687
T_base2ee[0,3] = 0.81692 # 平移向量的dx 
T_base2ee[1,3] = 0.19139 # 平移向量的dy 
T_base2ee[2,3] = 0.1687 # 平移向量的dz

# pcd = o3d.geometry.PointCloud()
# T_ee2pc = np.eye(4)
# T_ee2pc[ :3, :3] = pcd.get_rotation_matrix_from_xyz((-np.pi / 2, 0, -np.pi / 2)) # 旋转矩阵
# print("T_ee2pc:\n", T_ee2pc)

# -------------------------- 加载点云 ------------------------ 
# print("->正在加载点云... ") 
pcd = o3d.io.read_point_cloud("src/opencv_description/pointcloud_output/output.pcd") 
# print(pcd) 
# pcd.paint_uniform_color([1,0,0]) 
# print("->pcd质心：",pcd.get_center()) 
# # =========================================================== 
# # # -------------------------- transform ------------------------ 
# print("\n->点云的一般变换") 
# pcd_T = copy.deepcopy(pcd) 
# T_list = [0] * 7
# T1 = np.eye(4) 
# T1[ :3, :3] = pcd.get_rotation_matrix_from_xyz((0, np.pi/2, 0)) # 旋转矩阵
# T2 = np.eye(4) 
# T2[ :3, :3] = pcd.get_rotation_matrix_from_xyz((-np.pi/2, 0, 0)) # 旋转矩阵
# T3 = np.dot(T1, T2)
T_ee2image = np.eye(4) 
T_ee2image[ :3, :3] = pcd.get_rotation_matrix_from_xyz((-np.pi/2, np.pi/2, 0)) # 旋转矩阵
T_result = np.dot(T_base2ee, T_ee2image)

# R1 = T[:3, :3]
# T[0,3] = 0.0 # 平移向量的dx 
# T[1,3] = 0.0 # 平移向量的dy 
# T_list[0] = T
# print("\n->变换矩阵R：\n",R1) 
print("\n->变换矩阵T：\n",T_result)

P_image = np.asarray([-0.516,-0.585,1.53,1]).reshape(4,1)
P_base = np.dot(T_result, P_image)

P_image_2 = np.asarray([0.488,0.167,1.53,1]).reshape(4,1)
P_base_2 = np.dot(T_result, P_image_2)

print("\n->变换P：\n",P_base)
print("\n->变换P_2：\n",P_base_2)

# pcd_T.transform(T) 
# pcd_T.paint_uniform_color([0,0,1]) 
# print("\n->pcd_scale1质心：",pcd_T.get_center()) 
# =========================================================== 
# -------------------------- 可视化 -------------------------- 
# o3d.visualization.draw_geometries([pcd, pcd_T]) 

# import copy # 点云深拷贝 
# import open3d as o3d 
# import numpy as np 
# # -------------------------- 加载点云 ------------------------ 
# print("->正在加载点云... ") 
# pcd = o3d.io.read_point_cloud("bunny.pcd") 
# print(pcd) 
# pcd.paint_uniform_color([1,0,0]) 
# print("->pcd质心：",pcd.get_center()) 
# # =========================================================== # 
# # -------------------------- 点云旋转 ------------------------ 
# print("\n->采用欧拉角进行点云旋转") 
# pcd_EulerAngle = copy.deepcopy(pcd) 
# R1 = pcd.get_rotation_matrix_from_xyz((0,np.pi/2,0)) 
# print("旋转矩阵：\n",R1) 
# pcd_EulerAngle.rotate(R1,center = (0.1,0.1,0.1)) 
# # 指定旋转中心 
# pcd_EulerAngle.paint_uniform_color([0,0,1]) 
# print("\n->pcd_EulerAngle质心：",
# pcd_EulerAngle.get_center()) 
# # =========================================================== 
# # -------------------------- 可视化 -------------------------- 
# # o3d.visualization.draw_geometries([pcd, pcd_EulerAngle]) 
# # # ===========================================================

# import copy # 点云深拷贝 
# import open3d as o3d 
# import numpy as np 
# # -------------------------- 加载点云 ------------------------ 
# print("->正在加载点云... ") 
# pcd = o3d.io.read_point_cloud("bunny.pcd") 
# print(pcd) pcd.paint_uniform_color([1,0,0]) 
# print("->pcd质心：",pcd.get_center()) 
# # =========================================================== 
# # -------------------------- 点云旋转 ------------------------ 
# print("\n->采用欧拉角进行点云旋转") 
# pcd_EulerAngle = copy.deepcopy(pcd) 
# R1 = pcd.get_rotation_matrix_from_xyz((0,np.pi/2,0)) 
# print("旋转矩阵：\n",R1) 
# pcd_EulerAngle.rotate(R1) # 不指定旋转中心 
# pcd_EulerAngle.paint_uniform_color([0,0,1]) 
# print("\n->pcd_EulerAngle质心：",pcd_EulerAngle.get_center()) 
# # =========================================================== 
# #  -------------------------- 可视化 -------------------------- 
# # o3d.visualization.draw_geometries([pcd, pcd_EulerAngle]) 
# # # ===========================================================


