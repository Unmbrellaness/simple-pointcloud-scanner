# 使用官方demo点云对俩个原始点云进行点对点icp，点对面icp配准，并进行可视化，原始点云黄色，目标点云青色。
# python icp_registration.py
import copy

import numpy as np
import open3d as o3d


def draw_registration_result(source, target, transformation, title):
    source_temp = copy.deepcopy(source) # 制作点云深度拷贝保护原始点云
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0]) # 黄色
    target_temp.paint_uniform_color([0, 0.651, 0.929]) # 青色
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp], title="Open3D " + title)


def point_to_point_icp(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation, "\n")
    draw_registration_result(source, target, reg_p2p.transformation, 'point-to-point ICP')


def point_to_plane_icp(source, target, threshold, trans_init):
    print("Apply point-to-plane ICP")
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation, "\n")
    draw_registration_result(source, target, reg_p2l.transformation, 'point-to-plane ICP')


if __name__ == "__main__":
    # 本地没有会自动下载demo点云数据 https://github.com/isl-org/open3d_downloads/releases/download/20220301-data/DemoICPPointClouds.zip
    ply_path1 = r"data/p1_2000.ply"
    ply_path2 = r"data/p3_2000.ply"
    source = o3d.io.read_point_cloud(ply_path1)
    target = o3d.io.read_point_cloud(ply_path2)
    threshold = 2000
    trans_init = np.asarray([[ 9.98480503e-01 ,-4.80919118e-02 ,-2.69045252e-02 ,-2.69045237e+01],
                            [ 4.90520883e-02 , 9.98138339e-01 , 3.62456787e-02 ,-1.53483096e+01],
                            [ 2.51113141e-02, -3.75103267e-02 , 9.98980679e-01 , 3.28249766e+01],
                            [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])
    # draw_registration_result(source, target, trans_init, 'origin')

    print("Initial alignment")
    # 函数evaluate_registration计算两个主要指标：
    # fitness：其测量重叠区域（内部对应的数量/目标中的点的数量），越高越好。
    # inlier_rmse，其测量所有inlier对应的rmse相似性，越低越好。
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print("evaluation:" , evaluation, "\n")

    point_to_point_icp(source, target, threshold, trans_init)
    # point_to_plane_icp(source, target, threshold, trans_init)
