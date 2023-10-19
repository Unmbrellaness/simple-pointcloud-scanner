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
    
    # 可视化点云和标注原点
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd.colors[points[farthest_indices[0]]] = [1, 0, 0]
    # print(indices[0])
    pcd.colors[points[farthest_indices[1]]] = [0, 1, 0]
    pcd.colors[points[farthest_indices[2]]] = [0, 0, 1]
    # pcd.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(pcd.points), [0,0,0])))
    # pcd.colors[0] = [0,0,0]
    o3d.visualization.draw_geometries([pcd])

    return farthest_points


# 测试示例
pcd_file = r"C:\Users\10481\Desktop\PointCloud\data\p3_2000.ply"  # 替换为你的PCD文件路径
x, y, z = 0, 0, 0  # 给定点的坐标
farthest_points = find_farthest_points(pcd_file, x, y, z)
for i, point in enumerate(farthest_points):
    print("距离最远的点 {}:".format(i + 1))
    print("坐标: ({:.2f}, {:.2f}, {:.2f})".format(*point))
    print("---")