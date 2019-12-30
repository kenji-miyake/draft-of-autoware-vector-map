#pragma once

#include <sstream>
#include <string>

#include <ogrsf_frmts.h>

#include <autoware_vector_map/bridge/ogr/feature_conversion.h>
#include <autoware_vector_map/bridge/ogr/geometry_conversion.h>
#include <autoware_vector_map/traits/gpkg_contents.h>
#include <autoware_vector_map/traits/has_geometry.h>

namespace autoware_vector_map {
namespace util {

template <class T>
bool addIdString(const T& feature, std::stringstream* ss) {
  *ss << "id: " << feature.id << std::endl;
  return true;
}

template <class T, std::enable_if_t<traits::has_geometry<T>::value, std::nullptr_t> = nullptr>
bool addGeometryString(const T& feature, std::stringstream* ss) {
  using GeometryType = typename T::GeometryType;

  const auto ogr_geometry = bridge::toOgrGeometry<GeometryType>(feature.geometry);

  char* p_wkt = nullptr;
  ogr_geometry.exportToWkt(&p_wkt);

  *ss << "geometry: " << p_wkt << std::endl;

  return true;
}

template <class T, std::enable_if_t<!traits::has_geometry<T>::value, std::nullptr_t> = nullptr>
bool addGeometryString(const T& feature, std::stringstream* ss) {
  return false;
}

template <class T, size_t N,
          std::enable_if_t<traits::has_member_n<T, N>::value, std::nullptr_t> = nullptr>
bool addFieldString(const T& feature, std::stringstream* ss) {
  using member = traits::member_n<T, N>;
  *ss << member::name << ": " << feature.*member::reference << std::endl;
  return true;
}

template <class T, size_t N,
          std::enable_if_t<!traits::has_member_n<T, N>::value, std::nullptr_t> = nullptr>
bool addFieldString(const T& feature, std::stringstream* ss) {
  return false;
}

template <class T>
std::string toDebugString(const T& feature) {
  std::stringstream ss;

  ss << "class_name: " << traits::gpkg_content<T>::class_name() << std::endl;
  addIdString(feature, &ss);
  addGeometryString(feature, &ss);

  [&]() {
    // To be replaced by constexpr-if in C++17
    if (!addFieldString<T, 0>(feature, &ss)) return;
    if (!addFieldString<T, 1>(feature, &ss)) return;
    if (!addFieldString<T, 2>(feature, &ss)) return;
    if (!addFieldString<T, 3>(feature, &ss)) return;
    if (!addFieldString<T, 4>(feature, &ss)) return;
    if (!addFieldString<T, 5>(feature, &ss)) return;
    if (!addFieldString<T, 6>(feature, &ss)) return;
    if (!addFieldString<T, 7>(feature, &ss)) return;
    if (!addFieldString<T, 8>(feature, &ss)) return;
    if (!addFieldString<T, 9>(feature, &ss)) return;
    if (!addFieldString<T, 10>(feature, &ss)) return;
    if (!addFieldString<T, 11>(feature, &ss)) return;
    if (!addFieldString<T, 12>(feature, &ss)) return;
    if (!addFieldString<T, 13>(feature, &ss)) return;
    if (!addFieldString<T, 14>(feature, &ss)) return;
    if (!addFieldString<T, 15>(feature, &ss)) return;
    if (!addFieldString<T, 16>(feature, &ss)) return;
    if (!addFieldString<T, 17>(feature, &ss)) return;
    if (!addFieldString<T, 18>(feature, &ss)) return;
    if (!addFieldString<T, 19>(feature, &ss)) return;
    if (!addFieldString<T, 20>(feature, &ss)) return;
    if (!addFieldString<T, 21>(feature, &ss)) return;
    if (!addFieldString<T, 22>(feature, &ss)) return;
    if (!addFieldString<T, 23>(feature, &ss)) return;
    if (!addFieldString<T, 24>(feature, &ss)) return;
    if (!addFieldString<T, 25>(feature, &ss)) return;
    if (!addFieldString<T, 26>(feature, &ss)) return;
    if (!addFieldString<T, 27>(feature, &ss)) return;
    if (!addFieldString<T, 28>(feature, &ss)) return;
    if (!addFieldString<T, 29>(feature, &ss)) return;
    if (!addFieldString<T, 30>(feature, &ss)) return;
    if (!addFieldString<T, 31>(feature, &ss)) return;
    static_assert(!traits::has_member_n<T, 32>::value, "unsupported member size");
  }();

  return ss.str();
}

}  // namespace util
}  // namespace autoware_vector_map
