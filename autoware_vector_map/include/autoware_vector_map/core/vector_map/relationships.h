#pragma once

#include <string>

#include <autoware_vector_map/core.h>

namespace autoware_vector_map {

template <class T_LeftFeature, class T_RightFeature>
struct Relationship {
  using LeftFeatureType = T_LeftFeature;
  using RightFeatureType = T_RightFeature;

  Id id;
};

enum class RelationSide {
  Left,
  Right,
};

struct LaneSectionConnection : public Relationship<LaneSection, LaneSection> {
  Id lane_section_id;
  Id next_lane_section_id;
};

struct LaneConnection : public Relationship<Lane, Lane> {
  Id lane_id;
  Id next_lane_id;
};

struct AdjacentLane : public Relationship<Lane, Lane> {
  Id lane_id;
  Id adjacent_lane_id;
  std::string type;
};

struct Lane_StopLine : public Relationship<Lane, StopLine> {
  Id lane_id;
  Id stop_line_id;
};

struct Lane_Crosswalk : public Relationship<Lane, Crosswalk> {
  Id lane_id;
  Id crosswalk_id;
};

struct Lane_TrafficLight : public Relationship<Lane, TrafficLight> {
  Id lane_id;
  Id traffic_light_id;
};

}  // namespace autoware_vector_map
