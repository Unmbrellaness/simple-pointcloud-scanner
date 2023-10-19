import open3d
import numpy as np

pcd = open3d.geometry.PointCloud() # 首先建立一个pcd类型的数据（这是open3d中的数据形式）
np_points = np.random.rand(100, 3) # 随机生成点云

print(np_points)
# 将点云转换成open3d中的数据形式并用pcd来保存，以方便用open3d处理
pcd.points = open3d.utility.Vector3dVector(np_points)
open3d.visualization.draw_geometries([pcd])
# 将点云从oepn3d形式转换为矩阵形式
# np_points = np.asarray(pcd.points)
# print(np_points)
# 用open3d可视化生成的点云
# open3d.visualization.draw_geometries([pcd])