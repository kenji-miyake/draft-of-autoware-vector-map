#include <autoware_vector_map/api/vecor_map_api.h>

#include <autoware_vector_map/geometry/geometry.h>

#include <autoware_vector_map/util/helper_functions.h>
#include <autoware_vector_map/util/to_debug_string.h>

using autoware_vector_map::util::toDebugString;

namespace autoware_vector_map {
namespace api {

VectorMapApi::VectorMapApi(GpkgInterface* gpkg_interface) : gpkg_interface_((gpkg_interface)) {}

std::vector<Lane> VectorMapApi::findNearLanes(const Point3d& p, const double range) {
  return gpkg_interface_->findFeaturesByRange<Lane>(p, range);
}

std::optional<Lane> VectorMapApi::findNearestLane(const std::vector<Lane>& lanes,
                                                  const Point3d& p) {
  const auto distances =
      util::map(lanes, [&](const auto& lane) { return geometry::distance2d(lane.geometry, p); });

  return lanes.at(util::argmin(distances));
}

std::optional<Lane> VectorMapApi::findNearestLane(const Point3d& p, const double range) {
  const auto near_lanes = findNearLanes(p, range);
  if (near_lanes.empty()) {
    return {};
  }
  return findNearestLane(near_lanes, p);
}

std::vector<Lane> VectorMapApi::getNextLanes(const Lane& lane) {
  return gpkg_interface_->getRelatedFeaturesById<LaneConnection, RelationSide::Left>(lane.id);
}

std::vector<Lane> VectorMapApi::getPrevLanes(const Lane& lane) {
  return gpkg_interface_->getRelatedFeaturesById<LaneConnection, RelationSide::Right>(lane.id);
}

std::vector<StopLine> VectorMapApi::getRelatedStopLines(const Lane& lane) {
  return gpkg_interface_->getRelatedFeaturesById<Lane_StopLine, RelationSide::Left>(lane.id);
}

std::vector<Crosswalk> VectorMapApi::getRelatedCrosswalks(const Lane& lane) {
  return gpkg_interface_->getRelatedFeaturesById<Lane_Crosswalk, RelationSide::Left>(lane.id);
}

}  // namespace api
}  // namespace autoware_vector_map
