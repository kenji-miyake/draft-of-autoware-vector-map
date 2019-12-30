#pragma once

#include <autoware_vector_map/traits/gpkg_contents.h>

#define AUTOWARE_VECTOR_MAP_REGISTER_GPKG_RELATIONSHIP(RELATIONSHIP, TABLE_NAME, LEFT_FEATURE_ID, \
                                                       RIGHT_FEATURE_ID)                          \
  AUTOWARE_VECTOR_MAP_REGISTER_GPKG_CONTENT(RELATIONSHIP, TABLE_NAME)                             \
  AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(RELATIONSHIP, 0, LEFT_FEATURE_ID, invalid_id)          \
  AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(RELATIONSHIP, 1, RIGHT_FEATURE_ID, invalid_id)         \
                                                                                                  \
  template <>                                                                                     \
  constexpr const char* gpkg_relationship<RELATIONSHIP>::left_feature_id_name() {                 \
    return #LEFT_FEATURE_ID;                                                                      \
  }                                                                                               \
                                                                                                  \
  template <>                                                                                     \
  constexpr const char* gpkg_relationship<RELATIONSHIP>::right_feature_id_name() {                \
    return #RIGHT_FEATURE_ID;                                                                     \
  }                                                                                               \
                                                                                                  \
  template <>                                                                                     \
  inline Id gpkg_relationship<RELATIONSHIP>::left_feature_id(const RELATIONSHIP& relationship) {  \
    return relationship.LEFT_FEATURE_ID;                                                          \
  }                                                                                               \
                                                                                                  \
  template <>                                                                                     \
  inline Id gpkg_relationship<RELATIONSHIP>::right_feature_id(const RELATIONSHIP& relationship) { \
    return relationship.RIGHT_FEATURE_ID;                                                         \
  }

namespace autoware_vector_map {
namespace traits {

template <class T>
struct gpkg_relationship {
  static constexpr const char* left_feature_id_name();
  static constexpr const char* right_feature_id_name();
  static Id left_feature_id(const T& relationship);
  static Id right_feature_id(const T& relationship);

  template <RelationSide S, std::enable_if_t<S == RelationSide::Left, std::nullptr_t> = nullptr>
  static constexpr const char* this_feature_id_name() {
    return left_feature_id_name();
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Right, std::nullptr_t> = nullptr>
  static constexpr const char* this_feature_id_name() {
    return right_feature_id_name();
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Left, std::nullptr_t> = nullptr>
  static constexpr const char* related_feature_id_name() {
    return right_feature_id_name();
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Right, std::nullptr_t> = nullptr>
  static constexpr const char* related_feature_id_name() {
    return left_feature_id_name();
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Left, std::nullptr_t> = nullptr>
  static Id this_feature_id(const T& relationship) {
    return left_feature_id(relationship);
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Right, std::nullptr_t> = nullptr>
  static Id this_feature_id(const T& relationship) {
    return right_feature_id(relationship);
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Left, std::nullptr_t> = nullptr>
  static Id related_feature_id(const T& relationship) {
    return right_feature_id(relationship);
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Right, std::nullptr_t> = nullptr>
  static Id related_feature_id(const T& relationship) {
    return left_feature_id(relationship);
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Left, std::nullptr_t> = nullptr>
  static auto empty_this_feature() {
    return typename T::LeftFeatureType();
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Right, std::nullptr_t> = nullptr>
  static auto empty_this_feature() {
    return typename T::RightFeatureType();
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Left, std::nullptr_t> = nullptr>
  static auto empty_related_feature() {
    return typename T::RightFeatureType();
  }

  template <RelationSide S, std::enable_if_t<S == RelationSide::Right, std::nullptr_t> = nullptr>
  static auto empty_related_feature() {
    return typename T::LeftFeatureType();
  }

  template <RelationSide S>
  struct this_feature {
    using type = decltype(empty_this_feature<S>());
  };

  template <RelationSide S>
  struct related_feature {
    using type = decltype(empty_related_feature<S>());
  };

  template <RelationSide S>
  using this_feature_t = typename this_feature<S>::type;

  template <RelationSide S>
  using related_feature_t = typename related_feature<S>::type;
};

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_RELATIONSHIP(LaneSectionConnection, lane_section_connections,
                                               lane_section_id, next_lane_section_id)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_RELATIONSHIP(LaneConnection, lane_connections, lane_id,
                                               next_lane_id)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_RELATIONSHIP(AdjacentLane, adjacent_lanes, lane_id,
                                               adjacent_lane_id)
AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(AdjacentLane, 2, type, "")

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_RELATIONSHIP(Lane_StopLine, lanes_stop_lines, lane_id,
                                               stop_line_id)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_RELATIONSHIP(Lane_Crosswalk, lanes_crosswalks, lane_id,
                                               crosswalk_id)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_RELATIONSHIP(Lane_TrafficLight, lanes_traffic_lights, lane_id,
                                               traffic_light_id)
}  // namespace traits
}  // namespace autoware_vector_map
