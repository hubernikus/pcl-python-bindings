# for pytest
import numpy as np

from pcl.common import PointcloudXYZ, PointXYZ, PointCloud
from pcl.common import PointType as t
from pcl.segmentation import EuclideanClusterExtraction

EuclideanClusterExtractionXYZ = EuclideanClusterExtraction

def test_euclideanclusterextraction():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 0.0, 0.0))
    cloud.append(PointXYZ(0.1, 0.0, 0.0))
    cloud.append(PointXYZ(2.0, 0.0, 0.0))
    cloud.append(PointXYZ(2.1, 0.0, 0.0))
    cloud.append(PointXYZ(2.2, 0.0, 0.0))
    seg = EuclideanClusterExtractionXYZ()
    seg.setInputCloud(cloud)
    seg.setClusterTolerance(1.0)
    clusters = seg.extract()
    assert len(clusters[0]) == 3
    assert len(clusters[1]) == 2
    assert clusters[0][0] == 2
    assert clusters[0][1] == 3
    assert clusters[0][2] == 4
    assert clusters[1][0] == 0
    assert clusters[1][1] == 1


def test_cluster_general():
    cloud = PointCloud(t.PointXYZ)
    cloud['position'] = np.vstack((
        [0, 0.1, 2.0, 2.1, 2.2],
        np.zeros((2, 5))
    )).T

    assert len(cloud) == 5

    seg = EuclideanClusterExtraction(tolerance=1.0)
    labels = seg.extract(cloud)

    label_sets = [set(ll) for ll in labels]

    assert {2, 3, 4} in label_sets 
    assert {0, 1} in label_sets

