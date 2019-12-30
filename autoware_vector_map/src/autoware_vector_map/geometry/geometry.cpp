#include <autoware_vector_map/geometry/geometry.h>

#include <autoware_vector_map/bridge/boost/geometry_registration.h>

namespace autoware_vector_map {
namespace geometry {

std::vector<Point3d> intersection(const LineString3d& geom1, const LineString3d& geom2) {
  std::vector<Point3d> out;
  boost::geometry::intersection(geom1, geom2, out);

  for (auto& p : out) {
    p.z() = 0.0;
  }

  return out;
}

std::vector<Point2d> intersection(const LineString2d& geom1, const LineString2d& geom2) {
  std::vector<Point2d> out;
  boost::geometry::intersection(geom1, geom2, out);
  return out;
}

std::vector<Polygon2d> intersection(const LinearRing2d& geom1, const LinearRing2d& geom2) {
  std::vector<Polygon2d> out;
  boost::geometry::intersection(geom1, geom2, out);
  return out;
}

std::vector<Polygon2d> intersection(const Polygon2d& geom1, const Polygon2d& geom2) {
  std::vector<Polygon2d> out;
  boost::geometry::intersection(geom1, geom2, out);
  return out;
}

}  // namespace geometry
}  // namespace autoware_vector_map
