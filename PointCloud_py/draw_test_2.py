import open3d as o3d
import numpy as np
import copy

def draw_registration_result(source, target, title = "123"):
    source_temp = copy.deepcopy(source) # 制作点云深度拷贝保护原始点云
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0]) # 黄色
    target_temp.paint_uniform_color([0, 0.651, 0.929]) # 青色
    # source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp], title="Open3D " + title)

def tran_origin(pcd,x,y,z):
    tran_array = np.asarray([[1.0, 0.0, 0.0, -x], [0.0, 1.0, 0.0, -y],
                             [0.0, 0.0, 1.0, -z], [0.0, 0.0, 0.0, 1.0]])
    trans_temp = copy.deepcopy(pcd)
    trans_pcd = trans_temp.transform(tran_array)
    return trans_pcd

def find_farthest_point_kdtree(pcd,x,y,z):
    # pcd_temp = copy.deepcopy(pcd)
    # 构建kd树
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    # 寻找距离原点最远的3个点
    # 获取点云中点的数量
    num_points = len(pcd.points)

    [distances, indices,_] = pcd_tree.search_knn_vector_3d([x,y,z], num_points)
    # print(indices)
    # 创建一个新点
    new_point = [0, 0, 0]
    # 将新点添加到点云中
    pcd.points.append(new_point)

    # 可视化点云和标注原点
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd.colors[indices[-1]] = [1, 0, 0]
    # print(indices[0])
    pcd.colors[indices[-2]] = [0, 1, 0]
    pcd.colors[indices[-3]] = [0, 0, 1]
    # pcd.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(pcd.points), [0,0,0])))
    # pcd.colors[0] = [0,0,0]
    o3d.visualization.draw_geometries([pcd])

src_path = r"C:\Users\10481\Desktop\PointCloud\data\p3_2000.ply"
pcd = o3d.io.read_point_cloud(src_path)
tran_pcd = tran_origin(pcd,300,100,100)
# pcd.transform(tran_array)

# draw_registration_result(pcd,tran_pcd,"黄色为原图，青色为变换后的图")
find_farthest_point_kdtree(tran_pcd,0,0,0)
# o3d.visualization.draw(tran_pcd)


