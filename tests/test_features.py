# for pytest
from pcl.common import PointcloudXYZNormal, PointXYZNormal, PointCloud
from pcl.common import PointType as pt
from pcl.features import NormalEstimation, FPFHEstimation

from math import isnan

import numpy as np

# NormalEstimationXYZNormal = NormalEstimation


# def test_normalestimation():
#     cloud_normals = PointcloudXYZNormal()
#     # loadPCDFile("/home/markus/pcd_files/table_scene_mug_stereo_textured.pcd", cloud_normals)
#     for x in range(15):
#         for y in range(15):
#             cloud_normals.append(PointXYZNormal(x, y, 0.0))
#     assert len(cloud_normals) > 0
#     ne = NormalEstimationXYZNormal()
#     ne.setInputCloud(cloud_normals)
#     ne.setKSearch(50)
#     ne.compute(cloud_normals)
#     for point in cloud_normals:
#         if not isnan(point.x):
#             assert (
#                 abs(point.normal_x**2 + point.normal_y**2 + point.normal_z**2 - 1.0)
#                 < 1e-5
#             )


def test_normal_estimation_general():
    cloud_normals = PointCloud(pt.PointXYZ)
    num = 15
    cloud_normals["position"] = np.vstack(
        (np.arange(num), np.arange(num), np.zeros(num))
    ).T
    assert len(cloud_normals) == num

    ee = NormalEstimation(k_search=50, radius_search=0.0)
    ee.compute(cloud_normals)

    norm = np.sum(cloud_normals["normal"] ** 2, axis=1)
    np.testing.assert_allclose(norm, np.ones(num), atol=1e-6)


def test_fpfh_estimation():
    cloud = PointCloud(pt.PointNormal)
    side = 20
    
    xx = np.linspace(0, 1, side)
    yy = np.linspace(0, 1, side)
    xv, yv = np.meshgrid(xx, yy)

    cloud["position"] = np.column_stack((xv.flatten(), yv.flatten(), np.zeros_like(yv.flatten())))
    cloud["normal"] = np.zeros_like(cloud["position"])
    cloud["normal"][:, 2] = 1.0

    estimator = FPFHEstimation(radius_search=0.3)
    features = estimator.compute(cloud)

    assert np.max(features['histogram']) <= 100.1  , "Expected cap at 100%"
    assert features['histogram'].shape == (side*side, 33), "Expected to use FPFH33"

