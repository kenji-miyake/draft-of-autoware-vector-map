#pragma once

#include <string>

#include <autoware_vector_map/core.h>

namespace autoware_vector_map {

template <class T_Geometry>
struct Feature {
  using GeometryType = T_Geometry;

  Id id;
  T_Geometry geometry;
};

struct IntersectionArea : public Feature<Polygon3d> {};

struct LaneSection : public Feature<Polygon3d> {};

struct Lane : public Feature<LineString3d> {
  Id lane_section_id;
  double width;
  bool can_left_lane_change;
  bool can_right_lane_change;
  bool is_merge;
  bool is_diverge;
  bool is_intersection;
  bool is_left_turn;
  bool is_right_turn;
};

struct StopLine : public Feature<LineString3d> {
  bool is_reason_rule;
  bool is_reason_crosswalk;
  bool is_reason_traffic_light;
  bool is_reason_standby;
  bool is_reason_virtual;
};

struct Crosswalk : public Feature<LineString3d> {
  double width;
};

struct TrafficLight : public Feature<LineString3d> {};

}  // namespace autoware_vector_map
