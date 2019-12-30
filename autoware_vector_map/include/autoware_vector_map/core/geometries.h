#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_vector_map/core.h>

namespace autoware_vector_map {

struct Position : public Eigen::Vector3d {
  Position() = default;
  Position(const double x, const double y, const double z = 0.0) : Eigen::Vector3d(x, y, z) {}
};

struct Orientation : public Eigen::Quaterniond {
  using Eigen::Quaterniond::Quaterniond;
};

struct Pose {
  Position position;
  Orientation orientation;
};

struct Point2d;
struct Point3d;

struct LineString2d;
struct LineString3d;

struct LinearRing2d;
struct LinearRing3d;

struct Polygon2d;
struct Polygon3d;

struct Point2d : public Eigen::Vector2d {
  Point2d() = default;
  Point2d(const double x, const double y) : Eigen::Vector2d(x, y) {}

  Point3d to_3d(const double z = 0.0) const;
};

struct Point3d : public Eigen::Vector3d {
  Point3d() = default;
  Point3d(const double x, const double y, const double z = 0.0) : Eigen::Vector3d(x, y, z) {}

  Point2d to_2d() const;
};

struct LineString2d : public std::vector<Point2d> {
  using std::vector<Point2d>::vector;

  LineString3d to_3d(const double z = 0.0) const;
};

struct LineString3d : public std::vector<Point3d> {
  using std::vector<Point3d>::vector;

  LineString2d to_2d() const;
};

struct LinearRing2d : public LineString2d {
  using LineString2d::LineString2d;

  LinearRing3d to_3d(const double z = 0.0) const;
};

struct LinearRing3d : public LineString3d {
  using LineString3d::LineString3d;

  LinearRing2d to_2d() const;
};

struct Polygon2d {
  LinearRing2d exterior;
  std::vector<LinearRing2d> interiors;

  Polygon3d to_3d(const double z = 0.0) const;
};

struct Polygon3d {
  LinearRing3d exterior;
  std::vector<LinearRing3d> interiors;

  Polygon2d to_2d() const;
};

}  // namespace autoware_vector_map
