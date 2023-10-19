'''
Author: dongcidaci
Date: 2021-09-14 10:08:14
LastEditTime: 2021-09-14 10:31:45
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: \open3d_code\2_03_pointcloudpeizhun.py
'''
import open3d as o3d
import numpy as np
import copy
import time


# 点云全局配准
# 可视化
# 该辅助函数可以将配准的源点云和目标点云一起可视化
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


# 提取几何特征
# 降采样点云,估计法线,之后对每个点计算FPFH特征.FPFH特征是一个描述点的局部几何属性的33维的向量.
# 在3维空间中进行最近邻查询可以返回具有近似几何结构的点.
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


# 输入
# 以下代码从两个文件中读取源点云和目标点云.这一对点云使用单位矩阵作为初始矩阵之后是不对齐的.
def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
    target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


voxel_size = 0.5  # means 5cm for this dataset
source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)

print("1111111111111111",source_fpfh)

# RANSAC
# 使用RANSAC进行全局配准.在RANSAC迭代中，每次从源点云中选取　ransac_n 个随机点.
# 通过在3维FPFH特征空间中查询最邻近,可以在目标点云中找到他们的对应点.剪枝步骤需要使用快速修剪算法来提早拒绝错误匹配.
# Open3d提供以下剪枝算法:

# CorrespondenceCheckerBasedOnDistance检查对应的点云是否接近(也就是距离是否小于指定阈值)
# CorrespondenceCheckerBasedOnEdgeLength检查从源点云和目标点云对应中分别画上两条任意边(两个顶点连成的线)是否近似.
# CorrespondenceCheckerBasedOnNormal考虑的所有的对应的顶点法线的密切关系.他计算了两个法线向量的点积.使用弧度作为阈值.
# 只有通过剪枝步骤的匹配才用于转换,该转换将在整个点云上进行验证.
# 核心函数是 registration_ransac_based_on_feature_matching. RANSACConvergenceCriteria是里面一个十分重要的超参数.
# 他定义了RANSAC迭代的最大次数和验证的最大次数.这两个值越大,那么结果越准确,但同时也要花费更多的时间.
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            o3d.cpu.pybind.geometry.PointCloud(source_down),
            o3d.cpu.pybind.geometry.PointCloud(target_down),
            source_fpfh,
            target_fpfh,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3,
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print(result_ransac)
draw_registration_result(source_down, target_down, result_ransac.transformation)


# 局部优化
# 由于性能原因,全局配准只能在大规模降采样的点云上执行,配准的结果不够精细,我们使用 Point-to-plane ICP 去进一步优化配准结果.
def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


result_icp = refine_registration(source, target, source_fpfh, target_fpfh,
                                 voxel_size)
print(result_icp)
draw_registration_result(source, target, result_icp.transformation)

# 快速全局配准
# 由于无数的模型推荐和评估,导致基于RANSAC的全局配准需要很长的时间.
# 提出了一种加速的方法,该方法可以快速的优化几乎没有对应关系的线处理权重.这样在每次迭代的时候没有模型建议和评估,该方法就在计算的时候节约的大量的时间.
voxel_size = 0.05  # means 5cm for the dataset
source, target, source_down, target_down, source_fpfh, target_fpfh = \
    prepare_dataset(voxel_size)
# 基准
# 在下面代码中,我们将计时全局配准算法.

start = time.time()
result_ransac = execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
print("Global registration took %.3f sec.\n" % (time.time() - start))
print(result_ransac)
draw_registration_result(source_down, target_down,
                         result_ransac.transformation)


# 快速全局配准
# 我们采用和基准相同的输入
def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
          % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result


start = time.time()
result_fast = execute_fast_global_registration(source_down, target_down,
                                               source_fpfh, target_fpfh,
                                               voxel_size)
print("Fast global registration took %.3f sec.\n" % (time.time() - start))
print(result_fast)
draw_registration_result(source_down, target_down,
                         result_fast.transformation)