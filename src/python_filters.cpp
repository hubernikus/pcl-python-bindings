#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(pcl_filters_ext, m)
{
  // PassThrough not implemented since numpy can be used at similar speed
        
#if 0 // abstract class
  nb::class_<pcl::Filter<pcl::PointXYZ>(m, "FilterXYZ")
      .def(nb::init<>())
      .def("filter", &pcl::Filter<pcl::PointXYZ>::filter)
      ;
#endif
#if 0 // abstract class
  nb::class_<pcl::FilterIndices<pcl::PointXYZ>(m, "FilterIndicesXYZ")
      .def(nb::init<>())
      .def("setNegative", &pcl::FilterIndices<pcl::PointXYZ>::setNegative)
      ;
#endif
  nb::class_<pcl::PassThrough<pcl::PointXYZ>>(m, "PassThroughXYZ", "PassThrough passes points in a cloud based on constraints for one particular field of the point type. Iterates through the entire input once, automatically filtering non-finite points and the points outside the interval specified by setFilterLimits(), which applies only to the field specified by setFilterFieldName().")
      .def(nb::init<>())
      .def("setInputCloud", &pcl::PassThrough<pcl::PointXYZ>::setInputCloud, nb::arg("cloud"), "Provide the input dataset.")
      .def("setFilterLimits", &pcl::PassThrough<pcl::PointXYZ>::setFilterLimits, nb::arg("limit_min"), nb::arg("limit_max"), "Set the numerical limits for the field for filtering data. In conjunction with setFilterFieldName(), points having values outside this interval for this field will be discarded.")
      .def("setFilterFieldName", &pcl::PassThrough<pcl::PointXYZ>::setFilterFieldName, nb::arg("field_name"), "Provide the name of the field to be used for filtering data. In conjunction with setFilterLimits(), points having values outside this interval for this field will be discarded.")
      .def("setNegative", &pcl::FilterIndices<pcl::PointXYZ>::setNegative, nb::arg("negative"), "Set whether the regular conditions for points filtering should apply, or the inverted conditions.")
      .def("filter", nb::overload_cast<pcl::PointCloud<pcl::PointXYZ>&>(&pcl::Filter<pcl::PointXYZ>::filter), nb::arg("output"), "Calls the filtering method and returns the filtered dataset in output.") // TODO alternatively could return output, not as parameter?
      ;

  nb::class_<pcl::PassThrough<pcl::PointXYZRGBA>>(m, "PassThroughXYZRGBA")
      .def(nb::init<>())
      .def("setInputCloud", &pcl::PassThrough<pcl::PointXYZRGBA>::setInputCloud)
      .def("setFilterLimits", &pcl::PassThrough<pcl::PointXYZRGBA>::setFilterLimits)
      .def("setFilterFieldName",
           &pcl::PassThrough<pcl::PointXYZRGBA>::setFilterFieldName)
      .def("setNegative",
           &pcl::FilterIndices<pcl::PointXYZRGBA>::setNegative)
      .def("filter",
           nb::overload_cast<pcl::PointCloud<pcl::PointXYZRGBA>&>(
               &pcl::Filter<pcl::PointXYZRGBA>::filter))
      ;
}
