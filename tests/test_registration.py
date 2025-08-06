import pytest
from pcl.common import PointcloudXYZ, PointXYZ, PointCloud
from pcl.registration import IterativeClosestPointXYZ, IterativeClosestPoint

from pcl.common import PointType as t

import numpy as np


def test_iterativeclosestpoint():
    cloud_source = PointcloudXYZ()
    cloud_source.append(PointXYZ(0, 0, 0))
    cloud_source.append(PointXYZ(0, 10, 0))
    cloud_source.append(PointXYZ(0, 0, 10))
    cloud_target = PointcloudXYZ()
    cloud_target.append(PointXYZ(1, 0, 0))
    cloud_target.append(PointXYZ(1, 10, 0))
    cloud_target.append(PointXYZ(1, 0, 10))
    icp = IterativeClosestPointXYZ()
    icp.setInputSource(cloud_source)
    icp.setInputTarget(cloud_target)
    cloud_source_aligned = PointcloudXYZ()
    icp.align(cloud_source_aligned)
    assert len(cloud_source_aligned) == 3
    assert icp.hasConverged()
    transf = icp.getFinalTransformation()

    assert transf[0, 3] == pytest.approx(1.0)


def test_icp_general_cloud():
    source = PointCloud(t.PointXYZ)
    source["position"] = np.array([[0, 0, 0], [0, 10, 0], [0, 0, 10]])

    target = PointCloud(t.PointXYZ)
    target["position"] = np.array([[1, 0, 0], [1, 10, 0], [1, 0, 10]])

    icp = IterativeClosestPoint(
        # inlier_threshold=0.05,
        max_iterations=10,
        ransac_iterations=0,
    )

    icp.align(source=source, target=target)
    assert icp.converged

    expected_ = np.eye(4)
    expected_[0, 3] = 1.0
    np.testing.assert_allclose(icp.final_transformation, expected_, atol=1e-6)
