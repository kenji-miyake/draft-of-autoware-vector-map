#pragma once

#include <memory>
#include <vector>

#include <autoware_vector_map/future/optional.h>

#include <autoware_vector_map/core.h>
#include <autoware_vector_map/io/gpkg_interface.h>

#include <autoware_vector_map_msgs/BinaryGpkgMap.h>

namespace autoware_vector_map {
namespace api {

using autoware_vector_map::io::GpkgInterface;
using autoware_vector_map_msgs::BinaryGpkgMap;

class VectorMapApi {
 public:
  explicit VectorMapApi(GpkgInterface* gpkg_interface);

  template <class T>
  std::optional<T> getFeatureById(const Id id) {
    return gpkg_interface_->getFeatureById<T>(id);
  }

  template <class T>
  std::vector<T> getFeaturesByIds(const std::vector<Id>& ids) {
    return gpkg_interface_->getFeaturesByIds<T>(ids);
  }

  std::vector<Lane> findNearLanes(const Point3d& p, const double range);

  std::optional<Lane> findNearestLane(const std::vector<Lane>& lanes, const Point3d& p);
  std::optional<Lane> findNearestLane(const Point3d& p, const double range);

  std::vector<Lane> getNextLanes(const Lane& lane);
  std::vector<Lane> getPrevLanes(const Lane& lane);

  std::vector<StopLine> getRelatedStopLines(const Lane& lane);
  std::vector<Crosswalk> getRelatedCrosswalks(const Lane& lane);

 private:
  std::unique_ptr<GpkgInterface> gpkg_interface_;
};

}  // namespace api
}  // namespace autoware_vector_map
