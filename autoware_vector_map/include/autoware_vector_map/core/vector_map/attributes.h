#pragma once

#include <string>

#include <autoware_vector_map/core.h>

namespace autoware_vector_map {

template <class T_Feature>
struct Attribute {
  using FeatureType = T_Feature;

  Id id;
};

}  // namespace autoware_vector_map
