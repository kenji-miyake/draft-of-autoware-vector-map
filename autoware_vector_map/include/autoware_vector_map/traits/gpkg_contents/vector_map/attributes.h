#pragma once

#include <autoware_vector_map/traits/gpkg_contents.h>

#define AUTOWARE_VECTOR_MAP_REGISTER_GPKG_ATTRIBUTE(ATTRIBUTE, TABLE_NAME, FEATURE_ID) \
  AUTOWARE_VECTOR_MAP_REGISTER_GPKG_CONTENT(ATTRIBUTE, TABLE_NAME)                     \
  AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(ATTRIBUTE, 0, FEATURE_ID, invalid_id)       \
                                                                                       \
  template <>                                                                          \
  constexpr const char* gpkg_attribute<ATTRIBUTE>::feature_id_name() {                 \
    return #FEATURE_ID;                                                                \
  }

namespace autoware_vector_map {
namespace traits {

template <class T>
struct gpkg_attribute {
  static constexpr const char* feature_id_name();
};

}  // namespace traits
}  // namespace autoware_vector_map
