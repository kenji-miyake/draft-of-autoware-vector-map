#pragma once

#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <autoware_vector_map/core.h>

namespace autoware_vector_map {
namespace bridge {

template <class T>
struct ros_geometry;

template <class T>
using ros_geometry_t = typename ros_geometry<T>::type;

template <>
struct ros_geometry<Point3d> {
  using type = geometry_msgs::Point;
};

template <>
struct ros_geometry<LineString3d> {
  using type = std::vector<geometry_msgs::Point>;
};

template <>
struct ros_geometry<LinearRing3d> {
  using type = std::vector<geometry_msgs::Point>;
};

template <>
struct ros_geometry<Polygon3d> {
  using type = std::vector<geometry_msgs::Point>;
};

template <class T, class T_Ros = ros_geometry_t<T>>
T fromRosGeometry(const T_Ros& ros_geom);

template <class T, class T_Ros = ros_geometry_t<T>>
T_Ros toRosGeometry(const T& geom);

}  // namespace bridge
}  // namespace autoware_vector_map
