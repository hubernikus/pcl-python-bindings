# for pytest
from time import perf_counter

import numpy as np
from pcl.common import PointcloudXYZ, PointXYZ, PointCloud
from pcl.common import PointType as t


def test_pointcloud():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 1.0, 0.0))
    assert cloud[0].y == 1.0
    cloud.resize(5)
    assert len(cloud) == 5
    # cloud[0] = PointXYZ(0.0, 1.0, 0.0)
    # TODO expand


def test_general_cloud():
    cloud = PointCloud(t.PointXYZ)
    assert len(cloud) == 0
    position = np.array([[1, 2, 3], [4, 5, 6]])
    cloud["position"] = position
    result = cloud["position"]
    np.testing.assert_allclose(position, result)
    assert len(cloud) == position.shape[0]

    cloud = PointCloud(t.Normal)
    normal = np.array([[1, 2, 3], [4, 5, 6]])
    cloud["normal"] = normal
    result = cloud["normal"]
    np.testing.assert_allclose(normal, result)
    assert len(cloud) == normal.shape[0]


def test_keys():
    cloud = PointCloud(t.PointXYZ)
    assert {"position"} == set(cloud.keys())

    cloud = PointCloud(t.PointXYZI)
    assert {"position", "intensity"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZL)
    # assert {"position", "label"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZRGBA)
    # assert {"position", "color"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZRGB)
    # assert {"position", "color"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZRGBL)
    # assert {"position", "color", "label"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZLAB)
    # assert {"position", "Lab"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZHSV)
    # assert {"position", "hsv"} == set(cloud.keys())

    # cloud = PointCloud(t.Normal)
    # assert {"normal", "curvature"} == set(cloud.keys())

    cloud = PointCloud(t.PointNormal)
    assert {"position", "normal", "curvature"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZRGBNormal)
    # assert {"position", "color", "normal", "curvature"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZINormal)
    # assert {"position", "intensity", "normal", "curvature"} == set(cloud.keys())

    # cloud = PointCloud(t.PointXYZLNormal)
    # assert {"position", "label", "normal", "curvature"} == set(cloud.keys())


def everything_is_point(n_points: int):
    cloud = PointcloudXYZ()
    cloud.resize(n_points)

    for ii in range(n_points):
        cloud.append(PointXYZ(0.0, 0.0, ii))


def everything_is_cloud(n_points: int):
    cloud = PointCloud(t.PointXYZ)
    values = np.hstack(
        (np.zeros((n_points, 2)), np.arange(n_points).reshape(n_points, 1))
    )
    cloud["position"] = values


def compare_speed():
    """
    >>> compare_speed()

    Run with 1 point
    Point-Based took 0.002 ms
    Cloud-Based took 0.012 ms
    Run with 10 point
    Point-Based took 0.005 ms
    Cloud-Based took 0.011 ms
    Run with 100 point
    Point-Based took 0.028 ms
    Cloud-Based took 0.010 ms
    Run with 1000 point
    Point-Based took 0.275 ms
    Cloud-Based took 0.034 ms
    Run with 10000 point
    Point-Based took 2.011 ms
    Cloud-Based took 0.158 ms
    Run with 100000 point
    Point-Based took 20.600 ms
    Cloud-Based took 1.457 ms
    Run with 1000000 point
    Point-Based took 192.949 ms
    Cloud-Based took 14.528 ms
    """
    n_rep = 20
    for power in np.arange(7):
        n_points = int(10**power)
        print(f"Run with {n_points} point")
        point_time = 0
        cloud_time = 0
        for ii in range(n_rep):
            start = perf_counter()
            everything_is_point(n_points)
            stop = perf_counter()
            point_time += (stop - start) / n_rep

            start = perf_counter()
            everything_is_cloud(n_points)
            stop = perf_counter()
            cloud_time += (stop - start) / n_rep

        print(f"Point-Based took {point_time * 1000:0.3f} ms")
        print(f"Cloud-Based took {cloud_time * 1000:0.3f} ms")


if __name__ == "__main__":
    compare_speed()
