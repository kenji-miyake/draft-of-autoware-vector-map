#include <autoware_vector_map/bridge/ogr/geometry_conversion.h>

namespace autoware_vector_map {
namespace bridge {

template <>
Point3d fromOgrGeometry<Point3d>(OGRPoint* ogr_geom) {
  return Point3d(ogr_geom->getX(), ogr_geom->getY(), ogr_geom->getZ());
}

template <>
LineString3d fromOgrGeometry<LineString3d>(OGRLineString* ogr_geom) {
  LineString3d geom{};

  geom.reserve(ogr_geom->getNumPoints());
  for (int i = 0; i < ogr_geom->getNumPoints(); ++i) {
    geom.emplace_back(ogr_geom->getX(i), ogr_geom->getY(i), ogr_geom->getZ(i));
  }

  return geom;
}

template <>
LinearRing3d fromOgrGeometry<LinearRing3d>(OGRLinearRing* ogr_geom) {
  LinearRing3d geom{};

  geom.reserve(ogr_geom->getNumPoints());
  for (int i = 0; i < ogr_geom->getNumPoints(); ++i) {
    geom.emplace_back(ogr_geom->getX(i), ogr_geom->getY(i), ogr_geom->getZ(i));
  }

  return geom;
}

template <>
Polygon3d fromOgrGeometry<Polygon3d>(OGRPolygon* ogr_geom) {
  Polygon3d geom{};

  geom.exterior = fromOgrGeometry<LinearRing3d>(ogr_geom->getExteriorRing());

  geom.interiors.reserve(ogr_geom->getNumInteriorRings());
  for (int i = 0; i < ogr_geom->getNumInteriorRings(); ++i) {
    geom.interiors.push_back(fromOgrGeometry<LinearRing3d>(ogr_geom->getInteriorRing(i)));
  }

  return geom;
}

template <>
OGRPoint toOgrGeometry<Point3d>(const Point3d& geom) {
  return OGRPoint(geom.x(), geom.y(), geom.z());
}

template <>
OGRLineString toOgrGeometry<LineString3d>(const LineString3d& geom) {
  OGRLineString ogr_geom{};

  for (const auto& p : geom) {
    ogr_geom.addPoint(p.x(), p.y(), p.z());
  }

  return ogr_geom;
}

template <>
OGRLinearRing toOgrGeometry<LinearRing3d>(const LinearRing3d& geom) {
  OGRLinearRing ogr_geom{};

  for (const auto& p : geom) {
    ogr_geom.addPoint(p.x(), p.y(), p.z());
  }

  return ogr_geom;
}

template <>
OGRPolygon toOgrGeometry<Polygon3d>(const Polygon3d& geom) {
  OGRPolygon ogr_geom{};

  OGRLinearRing* exterior_ring = new OGRLinearRing();
  *exterior_ring = toOgrGeometry(geom.exterior);
  ogr_geom.addRingDirectly(exterior_ring);

  for (size_t i = 0; i < geom.interiors.size(); ++i) {
    OGRLinearRing* interior_ring = new OGRLinearRing();
    *interior_ring = toOgrGeometry(geom.interiors.at(i));
    ogr_geom.addRingDirectly(interior_ring);
  }

  return ogr_geom;
}

}  // namespace bridge
}  // namespace autoware_vector_map
