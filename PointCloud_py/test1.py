import open3d
import numpy as np


# 点云聚类测试
def open3d_cluster():
    # 读取点云文件
    pcd_path = r"data/p1_2000.ply"
    pcd = open3d.io.read_point_cloud(pcd_path)
    pcd = open3d.geometry.PointCloud(pcd)

    # 聚类距离设置为4，组成一类至少需要20个点
    labels = pcd.cluster_dbscan(eps=80, min_points=80, print_progress=True)
    max_label = max(labels)
    print(max_label)

    # 随机构建n+1种颜色，这里需要归一化
    colors = np.random.randint(1, 255, size=(max_label + 1, 3)) / 255.
    colors = colors[labels]             # 每个点云根据label确定颜色
    colors[np.array(labels) < 0] = 0    # 噪点配置为黑色
    pcd.colors = open3d.utility.Vector3dVector(colors)  # 格式转换(由于pcd.colors需要设置为Vector3dVector格式)

    # 可视化点云列表
    open3d.visualization.draw_geometries([pcd],
                                         window_name="cluster",
                                         width=800,
                                         height=600)


open3d_cluster()