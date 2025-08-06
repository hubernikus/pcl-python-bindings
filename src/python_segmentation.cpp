#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h> // TODO alternatively nb::bind_vector?

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include "bind_utils.hpp"

namespace nb = nanobind;
using namespace nb::literals;

using EuclideanClusterExtraction = pcl::EuclideanClusterExtraction<pcl::PointXYZ>;

NB_MODULE(pcl_segmentation_ext, m)
{
  nb::class_<EuclideanClusterExtraction>(m, "EuclideanClusterExtraction", 
    "EuclideanClusterExtraction represents a segmentation class for cluster extraction in an Euclidean sense.")
  .def("__init__", [](
    EuclideanClusterExtraction* extractor, 
    float tolerance, 
    float min_cluster_size, 
    float max_cluster_size) { 
      new (extractor) EuclideanClusterExtraction(); 
      extractor->setMinClusterSize(min_cluster_size);
      extractor->setMaxClusterSize(max_cluster_size);
      extractor->setClusterTolerance(tolerance);
    }, 
    "tolerance"_a=0.0, 
    "min_cluster_size"_a=1,
    "max_cluster_size"_a=((std::numeric_limits<int>::max)()) 
  )
  .def_prop_rw("tolerance", &EuclideanClusterExtraction::setClusterTolerance, &EuclideanClusterExtraction::getClusterTolerance, 
    "Set the spatial cluster tolerance as a measure in the L2 Euclidean space.")
  .def_prop_rw("min_cluster_size", &EuclideanClusterExtraction::setMinClusterSize, &EuclideanClusterExtraction::getMinClusterSize)
  .def_prop_rw("max_cluster_size", &EuclideanClusterExtraction::setMaxClusterSize, &EuclideanClusterExtraction::setMaxClusterSize)
  .def("extract", [](EuclideanClusterExtraction& ece, PointCloud& cloud){
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud.position_to_cloud(*pcl_cloud);
    ece.setInputCloud(pcl_cloud);

    std::vector< pcl::PointIndices > clusters2;
    ece.extract(clusters2);

    // TODO: Use python/nanobind data-structure like numpy-array directly
    std::vector<pcl::Indices> clusters; 
    clusters.reserve(clusters2.size());
    for(const auto& cluster: clusters2) { clusters.push_back(cluster.indices); }
    return clusters;
  }, "Cluster extraction in a PointCloud. Returns a list of lists of indices, so for example clusters[0] is the first cluster, and clusters[0][0] is the index of the first point in the first cluster.")
  // ;

  // nb::class_<pcl::EuclideanClusterExtraction<pcl::PointXYZ>>(m, "EuclideanClusterExtractionXYZ", "EuclideanClusterExtraction represents a segmentation class for cluster extraction in an Euclidean sense.")
  // .def(nb::init<>())
  .def("setInputCloud", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setInputCloud, nb::arg("cloud")) // TODO maybe inherit this from PCLBase?
  .def("setClusterTolerance", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setClusterTolerance, nb::arg("tolerance"), "Set the spatial cluster tolerance as a measure in the L2 Euclidean space.")
  .def("setMinClusterSize", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setMinClusterSize, nb::arg("min_cluster_size"), "Set the minimum number of points that a cluster needs to contain in order to be considered valid.")
  .def("setMaxClusterSize", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setMaxClusterSize, nb::arg("max_cluster_size"), "Set the maximum number of points that a cluster needs to contain in order to be considered valid.")
  //.def("extract", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::extract, nb::arg("clusters"), "Cluster extraction in a PointCloud.")
  .def("extract", [](pcl::EuclideanClusterExtraction<pcl::PointXYZ>& ece){
    std::vector< pcl::PointIndices > clusters2;
    ece.extract(clusters2);
    std::vector<pcl::Indices> clusters;
    clusters.reserve(clusters2.size());
    for(const auto& cluster: clusters2) { clusters.push_back(cluster.indices); }
    return clusters;
  }, "Cluster extraction in a PointCloud. Returns a list of lists of indices, so for example clusters[0] is the first cluster, and clusters[0][0] is the index of the first point in the first cluster.")
  ;
}
