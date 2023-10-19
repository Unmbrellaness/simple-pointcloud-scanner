# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# Copyright (c) 2018-2023 www.open3d.org
# SPDX-License-Identifier: MIT
# ----------------------------------------------------------------------------

import copy

import numpy as np
import open3d as o3d

import argparse

# 绘制配准结果
def draw_registration_result(source, target, transformation, title):
    source_temp = copy.deepcopy(source) # 制作点云深度拷贝保护原始点云
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0]) # 黄色
    target_temp.paint_uniform_color([0, 0.651, 0.929]) # 青色
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp], title="Open3D " + title)

# 点到点icp
def point_to_point_icp(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation, "\n")
    draw_registration_result(source, target, reg_p2p.transformation, 'point-to-point ICP')
    return reg_p2p

# 准备工作，传入参数为点云，降采样体素大小，函数返回降采样点云，fpfh点云特征
def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0,
                                             max_nn=30))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0,
                                             max_nn=100))
    return (pcd_down, pcd_fpfh)

# 粗配准，需要传入的参数有:图1地址，图2地址, 体素大小，距离乘系数，最大迭代数，可信度 ,返回粗配准的结果,重点是转换矩阵
def coarse_registration(src_path ,dst_path ,voxel_size, distance_multiplier,max_iterations,confidence):

    distance_threshold = distance_multiplier * voxel_size

    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
    print('Reading inputs')

    src = o3d.io.read_point_cloud(src_path)
    dst = o3d.io.read_point_cloud(dst_path)

    print('Downsampling inputs')
    src_down, src_fpfh = preprocess_point_cloud(src, voxel_size)
    dst_down, dst_fpfh = preprocess_point_cloud(dst, voxel_size)

    print('Running RANSAC')
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down,
        dst_down,
        src_fpfh,
        dst_fpfh,
        mutual_filter=True,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.
        TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            max_iterations, confidence))

    src.paint_uniform_color([1, 0, 0])
    dst.paint_uniform_color([0, 1, 0])
    # print(result.transformation)
    o3d.visualization.draw([src.transform(result.transformation), dst])
    return result

# 精配准，需要传入的参数:图1地址，图2地址，结果保存地址,体素大小，初始变换矩阵（粗配准结果），最大对应点对距离（threshold :Maximum correspondence points-pair distance）
def fine_registration(src_path ,dst_path, tar_path ,trans_init,threshold):
    source = o3d.io.read_point_cloud(src_path)
    target = o3d.io.read_point_cloud(dst_path)

    # trans_init = result.transformation  # 粗配准旋转矩阵
    # draw_registration_result(source, target, trans_init, 'origin')

    print("Initial alignment")
    # 函数evaluate_registration计算两个主要指标：
    # fitness：其测量重叠区域（内部对应的数量/目标中的点的数量），越高越好。
    # inlier_rmse，其测量所有inlier对应的rmse相似性，越低越好。
    evaluation = o3d.pipelines.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print("evaluation:", evaluation, "\n")

    result_trans = point_to_point_icp(source, target, threshold, trans_init)

    o3d.io.write_point_cloud(tar_path, source.transform(result_trans.transformation) + target)



# if __name__ == '__main__':
#
#     # 粗配准
#     src_path = "DemoICPPointClouds\cloud_bin_0.pcd"
#     dst_path = "DemoICPPointClouds\cloud_bin_1.pcd"
#     result_c_trans = coarse_registration(src_path, dst_path,voxel_size = 0.05,
#                                        distance_multiplier = 1.5,max_iterations = 100000, confidence=0.999)
#
#     # 精配准 并 保存
#     tar_path = "DemoICPPointClouds/result3.pcd"
#     fine_registration(src_path, dst_path, tar_path, result_c_trans.transformation, threshold = 0.02)



