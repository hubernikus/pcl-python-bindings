#include <string>

#include <nanobind/make_iterator.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "bind_utils.hpp"

namespace nb = nanobind;


NB_MODULE(pcl_common_ext, m)
{
  nb::enum_<PointType>(m, "PointType")
  .value("PointXYZ", PointXYZ)
  .value("PointXYZI", PointXYZI)
  .value("PointXYZL", PointXYZL)
  .value("PointXYZRGBA", PointXYZRGBA)
  .value("PointXYZRGB", PointXYZRGB)
  .value("PointXYZRGBL", PointXYZRGBL)
  .value("PointXYZLAB", PointXYZLAB)
  .value("PointXYZHSV", PointXYZHSV)
  .value("Normal", Normal)
  .value("PointNormal", PointNormal)
  .value("PointXYZRGBNormal", PointXYZRGBNormal)
  .value("PointXYZINormal", PointXYZINormal)
  .value("PointXYZLNormal", PointXYZLNormal)
  .export_values();

  nb::class_<PointCloud>(m, "PointCloud")
  .def(nb::init<>())
  .def(nb::init<PointType>())
  .def("__repr__", [](const PointCloud& cloud) {
    return nb::str("PointCloud<{}> of length {} with keys:\n {}").format(
      "", cloud.get_size(), cloud.get_keys());
  })
  .def("keys", &PointCloud::get_keys)
  .def("__len__", &PointCloud::get_size)
  .def("__getitem__", &PointCloud::get)
  .def("__setitem__", &PointCloud::set)
  ;

  nb::class_<pcl::PointXYZ>(m, "PointXYZ")
      .def(nb::init())
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
      .def_rw("x", &pcl::PointXYZ::x)
      .def_rw("y", &pcl::PointXYZ::y)
      .def_rw("z", &pcl::PointXYZ::z)
      .def("__repr__", [](const pcl::PointXYZ& p) {
        return nb::str("PointXYZ(x={}, y={}, z={})").format(p.x, p.y, p.z);
      });

  nb::class_<pcl::PointXYZRGBA>(m, "PointXYZRGBA")
      .def(nb::init())
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
      .def_rw("x", &pcl::PointXYZRGBA::x)
      .def_rw("y", &pcl::PointXYZRGBA::y)
      .def_rw("z", &pcl::PointXYZRGBA::z)
      .def("__repr__", [](const pcl::PointXYZRGBA& p) {
        return nb::str("PointXYZ(x={}, y={}, z={}, r={}, g={}, b={}, a={})").format(p.x, p.y, p.z, p.r, p.g, p.b, p.a);
      });
  
  nb::class_<pcl::PointNormal>(m, "PointXYZNormal")
      .def(nb::init())
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
      .def_rw("x", &pcl::PointNormal::x)
      .def_rw("y", &pcl::PointNormal::y)
      .def_rw("z", &pcl::PointNormal::z)
      .def_rw("normal_x", &pcl::PointNormal::normal_x)
      .def_rw("normal_y", &pcl::PointNormal::normal_y)
      .def_rw("normal_z", &pcl::PointNormal::normal_z)
      .def("__repr__", [](const pcl::PointNormal& p) {
        return nb::str("PointXYZNormal(x={}, y={}, z={})").format(p.x, p.y, p.z);
      });


  // TODO must avoid code duplication, maybe write macro?
  // TODO rename Pointcloud to PointCloud to match C++ name?
  nb::class_<pcl::PointCloud<pcl::PointXYZ>>(m, "PointcloudXYZ")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointXYZ>::push_back, nb::arg("point"), "Insert a new point in the cloud, at the end of the container.")
      .def("at", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZ>::at), "Get a point from the cloud")
      .def("__getitem__",
             [](pcl::PointCloud<pcl::PointXYZ> &cloud, std::size_t i) -> pcl::PointXYZ& {
                 return cloud[i];
             }
             //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
             )
      .def("__len__", [](const pcl::PointCloud<pcl::PointXYZ> &v) { return v.size(); })
      .def("resize", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZ>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointXYZ> &v) { v.clear(); })
      .def("__iter__",
        [](const pcl::PointCloud<pcl::PointXYZ> &v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointXYZ>>(), "iterator",
                                     v.begin(), v.end());
        }, nb::keep_alive<0, 1>());

  nb::class_<pcl::PointCloud<pcl::PointXYZRGBA>>(m, "PointcloudXYZRGBA")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointXYZRGBA>::push_back)
      .def("at",
           nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZRGBA>::at),
           "Get a point from the cloud")
      .def("__getitem__",
           [](pcl::PointCloud<pcl::PointXYZRGBA>& cloud,
              std::size_t i) -> pcl::PointXYZRGBA& {
             return cloud[i];
           }
           //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
           )
      .def("__len__", [](const pcl::PointCloud<pcl::PointXYZRGBA> &v) { return v.size(); })
      .def("resize",
           nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZRGBA>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointXYZRGBA> &v) { v.clear(); })
      .def(
          "__iter__",
          [](const pcl::PointCloud<pcl::PointXYZRGBA>& v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointXYZRGBA>>(),
                                     "iterator",
                                     v.begin(),
                                     v.end());
          },
          nb::keep_alive<0, 1>());
  nb::class_<pcl::PointCloud<pcl::PointNormal>>(m, "PointcloudXYZNormal")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointNormal>::push_back)
      .def("at", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointNormal>::at), "Get a point from the cloud")
      .def("__getitem__",
             [](pcl::PointCloud<pcl::PointNormal> &cloud, std::size_t i) -> pcl::PointNormal& {
                 return cloud[i];
             }
             //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
             )
      .def("__len__", [](const pcl::PointCloud<pcl::PointNormal> &v) { return v.size(); })
      .def("resize", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointNormal>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointNormal> &v) { v.clear(); })
      .def("__iter__",
        [](const pcl::PointCloud<pcl::PointNormal> &v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointNormal>>(), "iterator",
                                     v.begin(), v.end());
        }, nb::keep_alive<0, 1>());
}
