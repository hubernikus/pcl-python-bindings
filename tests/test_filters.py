import pytest
from pcl.common import PointcloudXYZ, PointXYZ, PointCloud
from pcl.common import PointType as t
from pcl.filters import PassThroughXYZ

import numpy as np


def test_passthrough():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 0.0, 0.0))
    cloud.append(PointXYZ(0.0, 0.0, 1.0))
    cloud.append(PointXYZ(0.0, 0.0, 2.0))
    filter = PassThroughXYZ()
    filter.setInputCloud(cloud)
    filter.setFilterFieldName("z")
    filter.setFilterLimits(0.5, 1.5)
    cloud_out = PointcloudXYZ()
    filter.filter(cloud_out)
    assert len(cloud_out) == 1
    assert cloud_out[0].z == 1.0


@pytest.mark.skip(reason="Filtering skipped")
def test_filters():
    cloud = PointCloud(t.PointXYZ)

    dim = 3
    cloud["position"] = np.hstack((np.zeros((dim, 2)), np.arange(dim).reshape(dim, 1)))

    lower = 0.5
    upper = 1.5
    pass_through = PassThrough(field_name="z", min=lower, max=upper)
    result = pass_through.filter(cloud)

    assert len(result) == 1
    assert len(cloud) == 3, "Input should not affected."

    # Filtering can be done in python-numpy with little overhead, too
    numpy_filtered = cloud['position']
    index = np.logical_and(numpy_filtered[:, 2] > lower, numpy_filtered[:, 2] < upper)
    numpy_filtered = numpy_filtered[index, :]

    np.testing.assert_allclose(result["position"], numpy_filtered)

