#include <memory>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include "bind_utils.hpp"

namespace nb = nanobind;
using namespace nb::literals;

using NormalEstimation = pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>;
using FPFHEstimation = pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>;

NB_MODULE(pcl_features_ext, m)
{
  nb::class_<NormalEstimation>(m, "NormalEstimation", 
    "NormalEstimation estimates local surface properties (surface normals and curvatures) at each 3D point.")
    .def("__init__", [](NormalEstimation* self, int k_search, float radius_search) { 
      new (self) NormalEstimation(); 
      self->setKSearch(k_search);
      self->setRadiusSearch(radius_search);
    }, "k_search"_a= 0, "radius_search"_a = 0.0)
  .def("setKSearch", &NormalEstimation::setKSearch)
  .def("getKSearch", &NormalEstimation::getKSearch)
  .def_prop_rw("k_search", &NormalEstimation::getKSearch, &NormalEstimation::setKSearch)  
  .def("setRadiusSearch", &NormalEstimation::setRadiusSearch)
  .def("getRadiusSearch", &NormalEstimation::getRadiusSearch)
  .def_prop_rw("radius_search", &NormalEstimation::getRadiusSearch, &NormalEstimation::setRadiusSearch)
  .def("compute", [](NormalEstimation& self, PointCloud& cloud){
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    cloud.position_to_cloud(*pcl_cloud);
    self.setInputCloud(pcl_cloud);
    self.compute(*pcl_cloud);
    cloud.normal_from_cloud(*pcl_cloud);
  }, "Estimate normals and curvatures.")
  ;

  // TODO: use multicore implementation 
  nb::class_<FPFHEstimation>(m, "FPFHEstimation", "Fast Point Feature Histogram Computation" )
  .def("__init__", [](FPFHEstimation* self, float radius_search) { 
    new (self) FPFHEstimation(); 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    self->setSearchMethod (tree);
    self->setRadiusSearch(radius_search);
  }, "radius_search"_a = 0.05)
  .def("setRadiusSearch", &FPFHEstimation::setRadiusSearch)
  .def("getRadiusSearch", &FPFHEstimation::getRadiusSearch)
  .def_prop_rw("radius_search", &FPFHEstimation::getRadiusSearch, &FPFHEstimation::setRadiusSearch)
  .def("compute", [](FPFHEstimation& self, PointCloud& cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr points (new pcl::PointCloud<pcl::PointXYZ>);
    cloud.position_to_cloud(*points);
    self.setInputCloud(points);
 
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
    cloud.normal_to_cloud(*normals);
    self.setInputNormals(normals);
 
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    self.compute(*fpfhs);

    PointCloud fpfh_cloud(PointType::FPFHSignature33); 
    fpfh_cloud.histogram_from_cloud(*fpfhs);
    return fpfh_cloud;
  })
  ;

}
