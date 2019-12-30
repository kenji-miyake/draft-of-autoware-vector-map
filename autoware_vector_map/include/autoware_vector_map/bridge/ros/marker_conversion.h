#pragma once

#include <vector>

#include <visualization_msgs/MarkerArray.h>

namespace autoware_vector_map {
namespace bridge {

geometry_msgs::Point createMarkerPosition(double x, double y, double z);
geometry_msgs::Quaternion createMarkerOrientation(double x, double y, double z, double w);
geometry_msgs::Vector3 createMarkerScale(double x, double y, double z);
std_msgs::ColorRGBA createMarkerColor(float r, float g, float b, float a);

template <class T>
visualization_msgs::Marker createMarker(const char* frame_id, const char* ns, const int32_t id,
                                        const T& geometry, const std_msgs::ColorRGBA& color);

}  // namespace bridge
}  // namespace autoware_vector_map
