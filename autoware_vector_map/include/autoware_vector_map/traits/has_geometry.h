#pragma once

#include <type_traits>

#include <autoware_vector_map/future/void_t.h>

namespace autoware_vector_map {
namespace traits {

template <class T, class = std::void_t<>>
struct has_geometry : std::false_type {};

template <class T>
struct has_geometry<T, std::void_t<decltype(std::declval<T>().geometry)>> : std::true_type {};

}  // namespace traits
}  // namespace autoware_vector_map
