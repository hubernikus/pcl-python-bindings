#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace nb = nanobind;

using NumpyArray2d = nb::ndarray<float, nb::numpy, nb::ndim<2>>;
using InputArray2d = nb::ndarray<float, nb::ndim<2>>;

// TODO: double check ownership - when setting and when returning
// TODO: how to handle different datatypes (np / tensorflow?)
// TODO: use array-view wherever needed (python-array) for speed

template<typename T>
inline nb::capsule delete_owner(T data){
  // Delete 'data' when the 'owner' capsule expires
  nb::capsule owner(data, [](void *p) noexcept {delete[] (float *) p;});
  return owner;
};

NumpyArray2d create_2d_array(size_t rows, size_t cols){
  float *data = new float[rows * cols];
  return NumpyArray2d(data, {rows, cols}, delete_owner(data));
};

enum PointType{
  PointXYZ,
  PointXYZI,
  PointXYZL,
  PointXYZRGBA,
  PointXYZRGB,
  PointXYZRGBL,
  PointXYZLAB,
  PointXYZHSV,
  Normal,
  PointNormal,
  PointXYZRGBNormal,
  PointXYZINormal,
  PointXYZLNormal,
  GENERAL // No restriction on properties
  // NUMBER_OF_POINTS // The number of enum-points for correct templatization
};

std::map<PointType, std::vector<std::string>> CLOUD_KEYS= {
  // TODO: add remaining points
  {PointType::PointXYZ, {"position"}},
  {PointType::PointXYZI, {"position", "intensity"}},
  {PointType::PointNormal, {"position", "normal", "curvature"}}
};

class PointCloud
{
  // Map of 2D-array with column first
  std::map<std::string, NumpyArray2d> _data = {};
  PointType _type = PointType::GENERAL;

  public:
    // TODO: constructor with different PointType's
    PointCloud(PointType type, size_t size = 0): _type(type){
      std::vector<std::string> keys = CLOUD_KEYS[type];
      for (auto it = keys.begin(); it != keys.end(); ++it){
        // TODO: varying width of arrays (?)
        _data[*it] = create_2d_array(size, 3);
        // _data["position"] = create_2d_array(3, size);
      }
    }

    bool size_consistent() const {
      /* Checks if the size of all arrays are consistent. */
      size_t size = _data.begin()->second.shape(0);
      for (auto it = _data.begin(); it != _data.end(); ++it){
        if (it->second.shape(0) != size){
          return false;
        }
      }
      return true;
    }

    void resize(size_t new_size){
      for (auto it = _data.begin(); it != _data.end(); ++it){
        size_t size = it->second.shape(0);
        if (size == new_size) {continue;}

        auto new_array = create_2d_array(new_size, it->second.shape(1));

        size_t n_row = std::min(size, new_size);
        for (size_t i = 0; i < n_row; ++i){
          for (size_t j = 0; j < new_array.shape(1); ++j){
            new_array(i, j) = it->second(i, j);
          }
        }
        _data[it->first] = new_array;
      } 
    }

    size_t get_size() const {
      if (!this->size_consistent()) {
        std::cout << "Size of cloud inconsistent\n";
        throw 101;}
      return _data.begin()->second.shape(0);
    }

    std::vector<std::string> get_keys() const {
      if (_type != PointType::GENERAL){
        return CLOUD_KEYS[_type];
      }
      
      std::vector<std::string> keys;
      for (auto it = _data.begin(); it != _data.end(); ++it)
      {
          // Add the key to the vector
          keys.push_back(it->first);
      }
      return keys;
    }

    template<typename CloudT>
    void position_from_cloud(CloudT& cloud){
      size_t cols = 3;
      auto rows = cloud.size(); 

      float* array = new float[rows * cols];
      for (size_t i = 0; i < rows; ++i){
        auto value = cloud[i];
        array[i * cols]  = value.x;
        array[i * cols + 1]  = value.y;
        array[i * cols + 2]  = value.z;
      }

      // TODO: check owner / deletion
      auto owner = delete_owner(array);
      auto ndarray = NumpyArray2d(array, {rows, cols}, owner);
      this->set("position", ndarray);
    };

    template<typename CloudT>
    void position_to_cloud(CloudT& cloud){
      auto ndarray = this->get("position");
      cloud.resize(ndarray.shape(0));

      for (int ii = 0; ii < ndarray.shape(0); ii++){
        cloud[ii].x = ndarray(ii, 0);
        cloud[ii].y = ndarray(ii, 1);
        cloud[ii].z = ndarray(ii, 2);
      }
    };


    template<typename CloudT>
    void normal_from_cloud(CloudT &cloud){
      size_t cols = 3;
      auto rows = cloud.size(); 
    
      float *_data = new float[rows * cols];
      for (size_t i = 0; i < rows; ++i){
        auto value = cloud[i];
        _data[i * cols]  = value.normal_x;
        _data[i * cols + 1]  = value.normal_y;
        _data[i * cols + 2]  = value.normal_z;
      }
    
      auto ndarray = NumpyArray2d(_data, {rows, cols}, delete_owner(_data));
      this->set("normal", ndarray);
    };

    template<typename CloudT>
    void normal_to_cloud(CloudT &cloud){
      auto ndarray = get("normal");
      cloud->resize(ndarray.shape(0));
    
      for (int ii = 0; ii < ndarray.shape(0); ii++){
        cloud[ii].normal_x = ndarray(ii, 0);
        cloud[ii].normal_y = ndarray(ii, 1);
        cloud[ii].normal_z = ndarray(ii, 2);
      }
    };

    // TODO: Add additional setters / getters for remaining features

    NumpyArray2d get(std::string key) {
      // TODO: check ownership (!)
      // TODO: different arrays needed (?)
      return _data[key];
    };

    void set(std::string key, NumpyArray2d value){
      // TODO: check existence / if it can be created
      // TODO: check array differences 
      // https://data-apis.org/array-api/latest/purpose_and_scope.html
      if (_type == PointType::GENERAL){
        if (_data.find(key) != _data.end()){
          std::cout << "Unexpected key: " << key << "\n";
          throw 101;
        }
      }
      _data[key] = value;
    };

    PointCloud slice(nb::slice slice) const {
      auto size = this->get_size();
      const auto& [start, stop, step, new_size] = slice.compute(size);
      std::vector<long int> indices(new_size); 

      long int value = start;
      for (auto it = 0; it < new_size; it++){
        indices[it] = value;
        value += step;
      }
      return this->get_subarray(indices);
    }

    PointCloud subarray_from_bool(nb::ndarray<bool, nb::ndim<1>> bool_indices) const {
      if (bool_indices.shape(0) != this->get_size()){
        std::cout << "Wrong indices size.\n";
        throw 101;
      }
      
      auto view = bool_indices.view();
      size_t new_size = 0;
      for (auto it = 0; it < view.shape(0); it++){
        if (view(it))
            new_size += 1;
      }

      std::vector<long int> indices(new_size); 
      size_t new_it = 0;
      for (auto it = 0; it < view.shape(0); it++){
        if (view(it)){
          indices[new_it] = it;
          new_it++;
        }
      }
      return this->get_subarray(indices);
    }

    template<typename T>
    PointCloud get_subarray(T indices) const{
      auto new_cloud = PointCloud(_type);
      for (auto it = _data.begin(); it != _data.end(); ++it){
        auto array = create_2d_array(indices.size(), it->second.shape(1));
        for (auto row= 0; row < indices.size(); row++){
          for (auto cc = 0; cc < it->second.shape(1); cc++){
            array(row, cc) = (float) it->second(indices[row], cc);
          }
        }
        new_cloud.set(it->first, array);
      }
      return new_cloud;
    }
};

