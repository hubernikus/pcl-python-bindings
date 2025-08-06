# for pytest
from pcl.common import PointcloudXYZNormal, PointXYZNormal, PointCloud
from pcl.common import PointType as t
from pcl.features import NormalEstimation

from math import isnan

import numpy as np

NormalEstimationXYZNormal = NormalEstimation


def test_normalestimation():
    cloud_normals = PointcloudXYZNormal()
    # loadPCDFile("/home/markus/pcd_files/table_scene_mug_stereo_textured.pcd", cloud_normals)
    for x in range(15):
        for y in range(15):
            cloud_normals.append(PointXYZNormal(x, y, 0.0))
    assert len(cloud_normals) > 0
    ne = NormalEstimationXYZNormal()
    ne.setInputCloud(cloud_normals)
    ne.setKSearch(50)
    ne.compute(cloud_normals)
    for point in cloud_normals:
        if not isnan(point.x):
            assert (
                abs(point.normal_x**2 + point.normal_y**2 + point.normal_z**2 - 1.0)
                < 1e-5
            )


def test_normal_estimation_general():
    cloud_normals = PointCloud(t.PointXYZ)
    num = 15
    cloud_normals["position"] = np.vstack(
        (np.arange(num), np.arange(num), np.zeros(num))
    ).T
    assert len(cloud_normals) == num

    ee = NormalEstimation(k_search=50, radius_search=0.0)
    ee.compute(cloud_normals)

    norm = np.sum(cloud_normals["normal"] ** 2, axis=1)
    np.testing.assert_allclose(norm, np.ones(num), atol=1e-6)

