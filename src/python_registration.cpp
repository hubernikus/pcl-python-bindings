#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include "bind_utils.hpp"

namespace nb = nanobind;
using namespace nb::literals;

using IterativeClosestPoint = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, double>;

NB_MODULE(pcl_registration_ext, m)
{
  nb::class_<IterativeClosestPoint>(m, "IterativeClosestPoint")
  .def("__init__", [](
    IterativeClosestPoint* icp, 
    int max_iterations, 
    float ransac_iterations
    // float max_correspondance_distance
    ) { 
      new (icp) IterativeClosestPoint(); 
      icp->setMaximumIterations(max_iterations);
      icp->setRANSACOutlierRejectionThreshold(ransac_iterations);
      // icp->setMaxCorrespondenceDistance(max_correspondance_distance);
    }, 
    "max_iterations"_a  = 10, 
    "ransac_iterations"_a = 0
    // "max_correspondance_distance"_a=0.05
  )
  // TODO remaining parameters
  // .def_prop_rw("max_correspondance_distance", &IterativeClosestPoint::setMaxCorrespondenceDistance, &IterativeClosestPoint::getMaxCorrespondenceDistance)
  .def_prop_rw("max_iterations", &IterativeClosestPoint::setMaximumIterations, &IterativeClosestPoint::getMaximumIterations)
  .def_prop_rw("ransac_iterations", &IterativeClosestPoint::setRANSACOutlierRejectionThreshold, &IterativeClosestPoint::getRANSACOutlierRejectionThreshold)
  // .def_prop_ro("euclidean_fitness_epsilon", &IterativeClosestPoint::)
  .def_prop_ro("converged", &IterativeClosestPoint::hasConverged, 
    "Return the state of convergence after the last align run.")
  .def_prop_ro("final_transformation", &IterativeClosestPoint::getFinalTransformation, 
    "Get the final transformation matrix estimated by the registration method.")
  .def("align", [](IterativeClosestPoint& icp, PointCloud& source, PointCloud& target){
    auto pcl_source = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    source.position_to_cloud(*pcl_source);
    icp.setInputSource(pcl_source);

    auto pcl_target = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    target.position_to_cloud(*pcl_target);
    icp.setInputTarget(pcl_target);

    auto pcl_output = pcl::PointCloud<pcl::PointXYZ>();
    icp.align(pcl_output);
  }, 
  "source"_a, 
  "target"_a,
  "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.") 
  ;

  nb::class_<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>>(m, "IterativeClosestPointXYZ") // TODO to discuss: Scalar=float or =double?
  .def(nb::init<>())
  .def("getFinalTransformation", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::getFinalTransformation, "Get the final transformation matrix estimated by the registration method.")
  .def("hasConverged", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::hasConverged, "Return the state of convergence after the last align run.")
  .def("setInputSource", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::setInputSource, nb::arg("cloud"), "Provide a pointer to the input source (e.g., the point cloud that we want to align to the target)")
  .def("setInputTarget", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::setInputTarget, nb::arg("cloud"), "Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)")
  .def("align", 
    nb::overload_cast
    <pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::PointCloudSource&>
    (&pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::align),
                nb::arg("output"), "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.") // TODO could alternatively return output?
  //.def("align", nb::overload_cast<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::PointCloudSource&, const pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4&>(&pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::align), nb::arg("output"), nb::arg("guess")=pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4::Identity(), "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.")
  ;
}
