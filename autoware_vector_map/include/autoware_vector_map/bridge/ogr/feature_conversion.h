#pragma once

#include <sstream>
#include <string>

#include <fmt/format.h>

#include <ogrsf_frmts.h>

#include <autoware_vector_map/bridge/ogr/geometry_conversion.h>
#include <autoware_vector_map/traits/gpkg_contents.h>
#include <autoware_vector_map/traits/has_geometry.h>

namespace autoware_vector_map {
namespace bridge {

template <class T>
struct OgrFieldCaster {
  static auto get_cast_function();

  static T cast(OGRFeature* ogr_feature, const int i) {
    return static_cast<T>(OgrFieldCaster<T>::get_cast_function()(ogr_feature, i));
  }
};

template <>
inline auto OgrFieldCaster<bool>::get_cast_function() {
  return std::mem_fn<int(int)>(&OGRFeature::GetFieldAsInteger);
}

template <>
inline auto OgrFieldCaster<int>::get_cast_function() {
  return std::mem_fn<int(int)>(&OGRFeature::GetFieldAsInteger);
}

template <>
inline auto OgrFieldCaster<int64_t>::get_cast_function() {
  return std::mem_fn<GIntBig(int)>(&OGRFeature::GetFieldAsInteger64);
}

template <>
inline auto OgrFieldCaster<float>::get_cast_function() {
  return std::mem_fn<double(int)>(&OGRFeature::GetFieldAsDouble);
}

template <>
inline auto OgrFieldCaster<double>::get_cast_function() {
  return std::mem_fn<double(int)>(&OGRFeature::GetFieldAsDouble);
}

template <>
inline auto OgrFieldCaster<const char*>::get_cast_function() {
  return std::mem_fn<const char*(int)>(&OGRFeature::GetFieldAsString);
}

template <>
inline auto OgrFieldCaster<std::string>::get_cast_function() {
  return std::mem_fn<const char*(int)>(&OGRFeature::GetFieldAsString);
}

template <class T>
bool assignId(OGRFeature* ogr_feature, T* feature) {
  feature->id = ogr_feature->GetFID();
  return true;
}

template <class T, std::enable_if_t<traits::has_geometry<T>::value, std::nullptr_t> = nullptr>
bool assignGeometry(OGRFeature* ogr_feature, T* feature) {
  using GeometryType = typename T::GeometryType;
  using OgrGeometryType = ogr_geometry_t<GeometryType>;

  OgrGeometryType* ogr_geometry = static_cast<OgrGeometryType*>(ogr_feature->GetGeometryRef());
  feature->geometry = fromOgrGeometry<GeometryType>(ogr_geometry);

  return true;
}

template <class T, std::enable_if_t<!traits::has_geometry<T>::value, std::nullptr_t> = nullptr>
bool assignGeometry(OGRFeature* ogr_feature, T* feature) {
  return false;
}

template <class T, size_t N,
          std::enable_if_t<traits::has_member_n<T, N>::value, std::nullptr_t> = nullptr>
bool assignField(OGRFeature* ogr_feature, T* feature) {
  using member = traits::member_n<T, N>;

  if (N >= ogr_feature->GetFieldCount()) {
    constexpr const char* class_name = traits::gpkg_content<T>::class_name();
    const auto msg = fmt::format("mismatch in field definition: {}", class_name);
    throw std::runtime_error(msg);
  }

  const char* field_name = ogr_feature->GetFieldDefnRef(N)->GetNameRef();
  const bool is_null = ogr_feature->IsFieldNull(N);

  assert(strcmp(field_name, member::name) == 0);

  if (is_null) {
    feature->*member::reference = member::if_null();
  } else {
    feature->*member::reference = OgrFieldCaster<typename member::type>::cast(ogr_feature, N);
  }

  return true;
}

template <class T, size_t N,
          std::enable_if_t<!traits::has_member_n<T, N>::value, std::nullptr_t> = nullptr>
bool assignField(OGRFeature* ogr_feature, T* feature) {
  return false;
}

template <class T>
bool assignFields(OGRFeature* ogr_feature, T* feature) {
  [&]() {
    // To be replaced by constexpr-if in C++17
    if (!assignField<T, 0>(ogr_feature, feature)) return;
    if (!assignField<T, 1>(ogr_feature, feature)) return;
    if (!assignField<T, 2>(ogr_feature, feature)) return;
    if (!assignField<T, 3>(ogr_feature, feature)) return;
    if (!assignField<T, 4>(ogr_feature, feature)) return;
    if (!assignField<T, 5>(ogr_feature, feature)) return;
    if (!assignField<T, 6>(ogr_feature, feature)) return;
    if (!assignField<T, 7>(ogr_feature, feature)) return;
    if (!assignField<T, 8>(ogr_feature, feature)) return;
    if (!assignField<T, 9>(ogr_feature, feature)) return;
    if (!assignField<T, 10>(ogr_feature, feature)) return;
    if (!assignField<T, 11>(ogr_feature, feature)) return;
    if (!assignField<T, 12>(ogr_feature, feature)) return;
    if (!assignField<T, 13>(ogr_feature, feature)) return;
    if (!assignField<T, 14>(ogr_feature, feature)) return;
    if (!assignField<T, 15>(ogr_feature, feature)) return;
    if (!assignField<T, 16>(ogr_feature, feature)) return;
    if (!assignField<T, 17>(ogr_feature, feature)) return;
    if (!assignField<T, 18>(ogr_feature, feature)) return;
    if (!assignField<T, 19>(ogr_feature, feature)) return;
    if (!assignField<T, 20>(ogr_feature, feature)) return;
    if (!assignField<T, 21>(ogr_feature, feature)) return;
    if (!assignField<T, 22>(ogr_feature, feature)) return;
    if (!assignField<T, 23>(ogr_feature, feature)) return;
    if (!assignField<T, 24>(ogr_feature, feature)) return;
    if (!assignField<T, 25>(ogr_feature, feature)) return;
    if (!assignField<T, 26>(ogr_feature, feature)) return;
    if (!assignField<T, 27>(ogr_feature, feature)) return;
    if (!assignField<T, 28>(ogr_feature, feature)) return;
    if (!assignField<T, 29>(ogr_feature, feature)) return;
    if (!assignField<T, 30>(ogr_feature, feature)) return;
    if (!assignField<T, 31>(ogr_feature, feature)) return;
    static_assert(!traits::has_member_n<T, 32>::value, "unsupported member size");
  }();

  return true;
}

template <class T>
T fromOgrFeature(OGRFeature* ogr_feature) {
  T feature{};

  assignId<T>(ogr_feature, &feature);
  assignGeometry<T>(ogr_feature, &feature);
  assignFields<T>(ogr_feature, &feature);

  return feature;
}

template <class T, size_t N,
          std::enable_if_t<traits::has_member_n<T, N>::value, std::nullptr_t> = nullptr>
bool addMemberName(std::stringstream* ss) {
  using member = traits::member_n<T, N>;
  *ss << ", " << member::name;
  return true;
}

template <class T, size_t N,
          std::enable_if_t<!traits::has_member_n<T, N>::value, std::nullptr_t> = nullptr>
bool addMemberName(std::stringstream* ss) {
  return false;
}

template <class T>
std::string createFeatureQuery(OGRLayer* layer) {
  const char* id_name = layer->GetFIDColumn();
  const char* geometry_name = layer->GetGeometryColumn();

  std::stringstream ss;

  [&]() {
    // To be replaced by constexpr-if in C++17
    if (!addMemberName<T, 0>(&ss)) return;
    if (!addMemberName<T, 1>(&ss)) return;
    if (!addMemberName<T, 2>(&ss)) return;
    if (!addMemberName<T, 3>(&ss)) return;
    if (!addMemberName<T, 4>(&ss)) return;
    if (!addMemberName<T, 5>(&ss)) return;
    if (!addMemberName<T, 6>(&ss)) return;
    if (!addMemberName<T, 7>(&ss)) return;
    if (!addMemberName<T, 8>(&ss)) return;
    if (!addMemberName<T, 9>(&ss)) return;
    if (!addMemberName<T, 10>(&ss)) return;
    if (!addMemberName<T, 11>(&ss)) return;
    if (!addMemberName<T, 12>(&ss)) return;
    if (!addMemberName<T, 13>(&ss)) return;
    if (!addMemberName<T, 14>(&ss)) return;
    if (!addMemberName<T, 15>(&ss)) return;
    if (!addMemberName<T, 16>(&ss)) return;
    if (!addMemberName<T, 17>(&ss)) return;
    if (!addMemberName<T, 18>(&ss)) return;
    if (!addMemberName<T, 19>(&ss)) return;
    if (!addMemberName<T, 20>(&ss)) return;
    if (!addMemberName<T, 21>(&ss)) return;
    if (!addMemberName<T, 22>(&ss)) return;
    if (!addMemberName<T, 23>(&ss)) return;
    if (!addMemberName<T, 24>(&ss)) return;
    if (!addMemberName<T, 25>(&ss)) return;
    if (!addMemberName<T, 26>(&ss)) return;
    if (!addMemberName<T, 27>(&ss)) return;
    if (!addMemberName<T, 28>(&ss)) return;
    if (!addMemberName<T, 29>(&ss)) return;
    if (!addMemberName<T, 30>(&ss)) return;
    if (!addMemberName<T, 31>(&ss)) return;
    static_assert(!traits::has_member_n<T, 32>::value, "unsupported member size");
  }();

  std::string geometry_part{};
  if (strcmp(geometry_name, "") != 0) {
    geometry_part = ", " + std::string(geometry_name);
  }

  return fmt::format("{}{}{}", id_name, geometry_part, ss.str());
}

}  // namespace bridge
}  // namespace autoware_vector_map
