import numpy
import open3d as o3d
import numpy as np
from copy import deepcopy


def draw_registration_result(source, target, transformation):
    # source_temp = copy.deepcopy(source)
    # target_temp = copy.deepcopy(target)
    source_temp = deepcopy(source)
    target_temp = deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])




# demo_icp_pcds = o3d.data.DemoICPPointClouds()
# source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
# target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

source = o3d.io.read_point_cloud("data/p1_2000.ply")
target = o3d.io.read_point_cloud("data/p3_2000.ply")

source.paint_uniform_color([1, 0.706, 0])
target.paint_uniform_color([0, 0.651, 0.929])
o3d.visualization.draw_geometries([source,target])

threshold = 0.02
# trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
#                          [-0.139, 0.967, -0.215, 0.7],
#                          [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
trans_init = numpy.asarray([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
draw_registration_result(source, target, trans_init)
