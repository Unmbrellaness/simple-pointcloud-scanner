import open3d as o3d
import numpy as np


def find_farthest_points(pcd_file, x, y, z):
    # 读取PCD文件
    pcd = o3d.io.read_point_cloud(pcd_file)

    # 获取点云数据
    points = np.asarray(pcd.points)

    # 计算点云中每个点与给定点的距离
    distances = np.linalg.norm(points - np.array([x, y, z]), axis=1)

    # 找到距离最远的三个点的索引
    farthest_indices = np.argsort(distances)[-3:]
    farthest_points = points[farthest_indices]

    return farthest_indices, farthest_points


# 测试示例
pcd_file = r"C:\Users\10481\Desktop\PointCloud\data\p3_2000.ply"  # 替换为你的PCD文件路径
x, y, z = 0, 0, 0  # 给定点的坐标
farthest_indices, farthest_points = find_farthest_points(pcd_file, x, y, z)

# 可视化点云和标记最远的点
pcd = o3d.io.read_point_cloud(pcd_file)
pcd.paint_uniform_color([0.7, 0.7, 0.7])  # 设置点云颜色为灰色

# 将最远的点颜色设置为黑色
for i, index in enumerate(farthest_indices):
    pcd.colors[index] = [0, 0, 0]

# 创建可视化窗口并添加点云
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)
vis.run()
vis.destroy_window()