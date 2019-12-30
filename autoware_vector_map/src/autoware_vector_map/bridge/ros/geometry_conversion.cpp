#include <autoware_vector_map/bridge/ros/geometry_conversion.h>

namespace autoware_vector_map {
namespace bridge {

template <>
Point3d fromRosGeometry<Point3d>(const geometry_msgs::Point& p) {
  return Point3d(p.x, p.y, p.z);
}

template <>
LineString3d fromRosGeometry<LineString3d>(const std::vector<geometry_msgs::Point>& ros_geom) {
  LineString3d geom{};

  geom.reserve(ros_geom.size());
  for (const auto& p : ros_geom) {
    geom.emplace_back(p.x, p.y, p.z);
  }

  return geom;
}

template <>
LinearRing3d fromRosGeometry<LinearRing3d>(const std::vector<geometry_msgs::Point>& ros_geom) {
  LinearRing3d geom{};

  geom.reserve(ros_geom.size());
  for (const auto& p : ros_geom) {
    geom.emplace_back(p.x, p.y, p.z);
  }

  return geom;
}

template <>
Polygon3d fromRosGeometry<Polygon3d>(const std::vector<geometry_msgs::Point>& ros_geom) {
  Polygon3d geom{};

  geom.exterior = fromRosGeometry<LinearRing3d>(ros_geom);

  return geom;
}

template <>
geometry_msgs::Point toRosGeometry<Point3d>(const Point3d& geom) {
  geometry_msgs::Point p;

  p.x = geom.x();
  p.y = geom.y();
  p.z = geom.z();

  return p;
}

template <>
std::vector<geometry_msgs::Point> toRosGeometry<LineString3d>(const LineString3d& geom) {
  std::vector<geometry_msgs::Point> ros_geom{};

  for (const auto& p : geom) {
    ros_geom.push_back(toRosGeometry<Point3d>(p));
  }

  return ros_geom;
}

template <>
std::vector<geometry_msgs::Point> toRosGeometry<LinearRing3d>(const LinearRing3d& geom) {
  std::vector<geometry_msgs::Point> ros_geom{};

  for (const auto& p : geom) {
    ros_geom.push_back(toRosGeometry<Point3d>(p));
  }

  return ros_geom;
}

template <>
std::vector<geometry_msgs::Point> toRosGeometry<Polygon3d>(const Polygon3d& geom) {
  std::vector<geometry_msgs::Point> ros_geom{};

  for (const auto& p : geom.exterior) {
    ros_geom.push_back(toRosGeometry<Point3d>(p));
  }

  return ros_geom;
}

}  // namespace bridge
}  // namespace autoware_vector_map
