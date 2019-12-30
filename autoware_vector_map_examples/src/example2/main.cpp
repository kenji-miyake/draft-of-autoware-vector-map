#include <examples_common.h>

#include <autoware_vector_map/api/vecor_map_api.h>

#include <autoware_vector_map/util/helper_functions.h>

namespace autoware_vector_map {

using util::contains_all;
using util::equal;
using util::sorted;
using util::to_ids;

void example(const char* vector_map_path) {
  api::VectorMapApi vmap(new io::GpkgInterface(vector_map_path));

  // findNearestLane
  {
    const Point3d p(-5022.11625494, -41829.62138366, 0);
    const double range = 5.0;
    const auto nearest_lane = vmap.findNearestLane(p, range);
    assert(nearest_lane->id == 2);
  }

  // getNextLanes / getPrevLanes
  {
    const auto lane = *vmap.getFeatureById<Lane>(58);
    const auto next_lanes = vmap.getNextLanes(lane);
    const auto prev_lanes = vmap.getPrevLanes(lane);
    assert(equal(sorted(to_ids(next_lanes)), {20, 52, 53}));
    assert(equal(sorted(to_ids(prev_lanes)), {}));
  }

  // getRelatedStopLines / getRelatedCrosswalks
  {
    const auto lane = *vmap.getFeatureById<Lane>(20);
    const auto stop_lines = vmap.getRelatedStopLines(lane);
    const auto crosswalks = vmap.getRelatedCrosswalks(lane);
    assert(equal(sorted(to_ids(stop_lines)), {2}));
    assert(equal(sorted(to_ids(crosswalks)), {2, 4}));
  }
}

}  // namespace autoware_vector_map

int main(int argc, char* argv[]) {
  autoware_vector_map::example(helper::getExampleFilePath().c_str());
}
