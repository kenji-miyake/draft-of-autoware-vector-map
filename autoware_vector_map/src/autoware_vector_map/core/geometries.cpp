#include <autoware_vector_map/core.h>

namespace autoware_vector_map {

Point3d Point2d::to_3d(const double z) const { return Point3d(this->x(), this->y(), z); }

Point2d Point3d::to_2d() const { return Point2d(this->x(), this->y()); }

LineString3d LineString2d::to_3d(const double z) const {
  LineString3d g_out;
  g_out.reserve(this->size());

  std::transform(std::cbegin(*this), std::cend(*this), std::back_inserter(g_out),
                 [&](const auto& p) { return p.to_3d(z); });

  return g_out;
}

LineString2d LineString3d::to_2d() const {
  LineString2d g_out;
  g_out.reserve(this->size());

  std::transform(std::cbegin(*this), std::cend(*this), std::back_inserter(g_out),
                 [&](const auto& p) { return p.to_2d(); });

  return g_out;
}

LinearRing3d LinearRing2d::to_3d(const double z) const {
  LinearRing3d g_out;
  g_out.reserve(this->size());

  std::transform(std::cbegin(*this), std::cend(*this), std::back_inserter(g_out),
                 [&](const auto& p) { return p.to_3d(z); });

  return g_out;
}

LinearRing2d LinearRing3d::to_2d() const {
  LinearRing2d g_out;
  g_out.reserve(this->size());

  std::transform(std::cbegin(*this), std::cend(*this), std::back_inserter(g_out),
                 [&](const auto& p) { return p.to_2d(); });

  return g_out;
}

Polygon3d Polygon2d::to_3d(const double z) const {
  Polygon3d g_out;

  g_out.exterior = this->exterior.to_3d(z);

  g_out.interiors.reserve(this->interiors.size());
  for (const auto& interior : this->interiors) {
    g_out.interiors.push_back(interior.to_3d(z));
  }

  return g_out;
}

Polygon2d Polygon3d::to_2d() const {
  Polygon2d g_out;

  g_out.exterior = this->exterior.to_2d();

  g_out.interiors.reserve(this->interiors.size());
  for (const auto& interior : this->interiors) {
    g_out.interiors.push_back(interior.to_2d());
  }

  return g_out;
}
}  // namespace autoware_vector_map
