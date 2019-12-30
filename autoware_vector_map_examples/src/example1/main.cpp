
#include <examples_common.h>

#include <autoware_vector_map/io/gpkg_interface.h>

#include <autoware_vector_map/bridge/boost/geometry_registration.h>
#include <autoware_vector_map/util/to_debug_string.h>

namespace autoware_vector_map {

void printTitle(const char* title) { std::cout << "### " << title << " ###" << std::endl; }

void example(const char* vector_map_path) {
  io::GpkgInterface gpkg_interface(vector_map_path);

  {
    printTitle("getFeatureById");

    using Feature = Lane;

    for (Id id : {-1, 0, 1, 3, 1000}) {
      const auto feature = gpkg_interface.getFeatureById<Feature>(id);
      if (feature) {
        std::cout << util::toDebugString(*feature) << std::endl;
      } else {
        std::cout << "feature not exist: " << id << std::endl;
      }
    }
  }

  {
    printTitle("getFeaturesByIds");

    using Feature = LaneSection;
    const std::vector<Id> ids{-1, 0, 2, 4, 1000};

    const auto features = gpkg_interface.getFeaturesByIds<Feature>(ids);
    for (const auto& feature : features) {
      std::cout << util::toDebugString(feature) << std::endl;
    }
  }

  {
    printTitle("getAllFeatures");

    using Feature = Crosswalk;

    const auto features = gpkg_interface.getAllFeatures<Feature>();
    for (const auto& feature : features) {
      std::cout << util::toDebugString(feature) << std::endl;
    }
  }

  {
    printTitle("findFeaturesByRange");

    using Feature = Lane;
    const Point3d p(80721.15, 7393.34, 0);
    const double range = 5.0;

    const auto features = gpkg_interface.findFeaturesByRange<Feature>(p, range);
    for (const auto& feature : features) {
      std::cout << util::toDebugString(feature) << std::endl;
    }
  }

  {
    printTitle("getRelatedFeaturesById");

    using Relationship = LaneConnection;
    const auto lane = *gpkg_interface.getFeatureById<Lane>(1);

    const auto features =
        gpkg_interface.getRelatedFeaturesById<Relationship, RelationSide::Left>(lane.id);
    for (const auto& feature : features) {
      std::cout << util::toDebugString(feature) << std::endl;
    }
  }

  {
    printTitle("getRelatedFeaturesById(with predicate)");

    using Relationship = AdjacentLane;
    const auto lane = *gpkg_interface.getFeatureById<Lane>(30);

    const auto features1 = gpkg_interface.getRelatedFeaturesById<Relationship, RelationSide::Left>(
        lane.id, [](const Relationship& r) { return r.type == "left"; });
    for (const auto& feature : features1) {
      std::cout << util::toDebugString(feature) << std::endl;
    }

    const auto features2 = gpkg_interface.getRelatedFeaturesById<Relationship, RelationSide::Left>(
        lane.id, [](const Relationship& r) { return r.type == "right"; });
    for (const auto& feature : features2) {
      std::cout << util::toDebugString(feature) << std::endl;
    }
  }

  {
    printTitle("boost::geometry integration");

    const auto lane_1 = *gpkg_interface.getFeatureById<Lane>(1);
    const auto lane_2 = *gpkg_interface.getFeatureById<Lane>(2);

    std::cout << boost::geometry::distance(lane_1.geometry.at(0), lane_2.geometry) << std::endl;
  }
}
}  // namespace autoware_vector_map

int main(int argc, char* argv[]) {
  autoware_vector_map::example(helper::getExampleFilePath().c_str());
}
