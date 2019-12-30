#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/geometry/geometries/register/multi_point.hpp>
#include <boost/geometry/geometries/register/multi_polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometries/register/segment.hpp>

#include <autoware_vector_map/core.h>

#define AUTOWARE_VECTOR_MAP_REGISTER_BOOST_RING(LINEAR_RING, ORDER_SELECTOR, CLOSURE_SELECTOR) \
  BOOST_GEOMETRY_REGISTER_RING(LINEAR_RING)                                                    \
                                                                                               \
  namespace boost {                                                                            \
  namespace geometry {                                                                         \
  namespace traits {                                                                           \
  template <>                                                                                  \
  struct point_order<LINEAR_RING> {                                                            \
    static const order_selector value = ORDER_SELECTOR;                                        \
  };                                                                                           \
                                                                                               \
  template <>                                                                                  \
  struct closure<LINEAR_RING> {                                                                \
    static const closure_selector value = CLOSURE_SELECTOR;                                    \
  };                                                                                           \
  }                                                                                            \
  }                                                                                            \
  }

#define AUTOWARE_VECTOR_MAP_REGISTER_BOOST_POLYGON(POLYGON, EXTERIOR, INTERIORS) \
  namespace boost {                                                              \
  namespace geometry {                                                           \
  namespace traits {                                                             \
  template <>                                                                    \
  struct tag<POLYGON> {                                                          \
    using type = polygon_tag;                                                    \
  };                                                                             \
                                                                                 \
  template <>                                                                    \
  struct ring_const_type<POLYGON> {                                              \
    using type = const decltype(std::declval<POLYGON>().EXTERIOR)&;              \
  };                                                                             \
                                                                                 \
  template <>                                                                    \
  struct ring_mutable_type<POLYGON> {                                            \
    using type = decltype(std::declval<POLYGON>().EXTERIOR)&;                    \
  };                                                                             \
                                                                                 \
  template <>                                                                    \
  struct interior_const_type<POLYGON> {                                          \
    using type = const decltype(std::declval<POLYGON>().INTERIORS)&;             \
  };                                                                             \
                                                                                 \
  template <>                                                                    \
  struct interior_mutable_type<POLYGON> {                                        \
    using type = decltype(std::declval<POLYGON>().INTERIORS)&;                   \
  };                                                                             \
                                                                                 \
  template <>                                                                    \
  struct exterior_ring<POLYGON> {                                                \
    static auto& get(POLYGON& p) { return p.EXTERIOR; }                          \
    static const auto& get(const POLYGON& p) { return p.EXTERIOR; }              \
  };                                                                             \
                                                                                 \
  template <>                                                                    \
  struct interior_rings<POLYGON> {                                               \
    static auto& get(POLYGON& p) { return p.INTERIORS; }                         \
    static const auto& get(const POLYGON& p) { return p.INTERIORS; }             \
  };                                                                             \
  }                                                                              \
  }                                                                              \
  }

// clang-format off
BOOST_GEOMETRY_REGISTER_POINT_2D(autoware_vector_map::Point2d, double, cs::cartesian, x(), y())
BOOST_GEOMETRY_REGISTER_POINT_3D(autoware_vector_map::Point3d, double, cs::cartesian, x(), y(), z())

BOOST_GEOMETRY_REGISTER_LINESTRING(autoware_vector_map::LineString2d)
BOOST_GEOMETRY_REGISTER_LINESTRING(autoware_vector_map::LineString3d)

AUTOWARE_VECTOR_MAP_REGISTER_BOOST_RING(autoware_vector_map::LinearRing2d, counterclockwise, closed)
AUTOWARE_VECTOR_MAP_REGISTER_BOOST_RING(autoware_vector_map::LinearRing3d, counterclockwise, closed)

AUTOWARE_VECTOR_MAP_REGISTER_BOOST_POLYGON(autoware_vector_map::Polygon2d, exterior, interiors)
AUTOWARE_VECTOR_MAP_REGISTER_BOOST_POLYGON(autoware_vector_map::Polygon3d, exterior, interiors)

// clang-format on
