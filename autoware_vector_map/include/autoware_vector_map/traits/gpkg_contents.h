#pragma once

#include <autoware_vector_map/future/void_t.h>

#include <autoware_vector_map/core.h>

#define AUTOWARE_VECTOR_MAP_REGISTER_GPKG_CONTENT(CONTENT, TABLE_NAME) \
  template <>                                                          \
  constexpr const char* gpkg_content<CONTENT>::class_name() {          \
    return #CONTENT;                                                   \
  }                                                                    \
                                                                       \
  template <>                                                          \
  constexpr const char* gpkg_content<CONTENT>::table_name() {          \
    return #TABLE_NAME;                                                \
  }

#define AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(FEATURE, INDEX, VARIABLE, IF_NULL) \
  template <>                                                                       \
  template <>                                                                       \
  struct gpkg_content<FEATURE>::member_def<INDEX> {                                 \
    using type = decltype(FEATURE::VARIABLE);                                       \
    static constexpr const char* name = #VARIABLE;                                  \
    static constexpr auto reference = &FEATURE::VARIABLE;                           \
    static type if_null() { return IF_NULL; }                                       \
  };

namespace autoware_vector_map {
namespace traits {

template <class T>
struct gpkg_content {
  static constexpr const char* class_name();
  static constexpr const char* table_name();

  // The order of member_def<N> should be the same as the table's one
  template <size_t N>
  struct member_def;
};

template <class T, size_t N>
using member_n = typename gpkg_content<T>::template member_def<N>;

template <class T, size_t N, class = std::void_t<>>
struct has_member_n : std::false_type {};

template <class T, size_t N>
struct has_member_n<T, N, std::void_t<typename member_n<T, N>::type>> : std::true_type {};

}  // namespace traits
}  // namespace autoware_vector_map

#include "gpkg_contents/vector_map/attributes.h"
#include "gpkg_contents/vector_map/features.h"
#include "gpkg_contents/vector_map/relationships.h"
