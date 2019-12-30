#include "vector_map_visualizer_node.h"

#include <memory>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include <autoware_vector_map/bridge/ros/marker_conversion.h>
#include <autoware_vector_map/bridge/ros/message_conversion.h>
#include <autoware_vector_map/io/gpkg_interface.h>

namespace avm = autoware_vector_map;
using autoware_vector_map::io::GpkgInterface;
using autoware_vector_map::traits::gpkg_content;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

template <class T>
void addMarkers(const std::unique_ptr<GpkgInterface>& gpkg_interface, MarkerArray* marker_array,
                const std_msgs::ColorRGBA& color,
                const std::function<void(const T&, Marker*)> post_process = nullptr,
                const char* ns = gpkg_content<T>::class_name()) {
  using autoware_vector_map::bridge::createMarker;

  const auto features = gpkg_interface->getAllFeatures<T>();
  for (const auto& feature : features) {
    auto marker =
        createMarker("map", ns, static_cast<int32_t>(feature.id), feature.geometry, color);

    if (post_process) {
      post_process(feature, &marker);
    }

    marker_array->markers.push_back(marker);
  }
}

MarkerArray createAllMarkers(const std::unique_ptr<GpkgInterface>& gpkg_interface) {
  using autoware_vector_map::bridge::createMarkerColor;
  using autoware_vector_map::bridge::createMarkerScale;

  MarkerArray marker_array;

  addMarkers<avm::IntersectionArea>(gpkg_interface, &marker_array,
                                    createMarkerColor(0.7, 0.0, 0.0, 0.5));

  addMarkers<avm::LaneSection>(gpkg_interface, &marker_array,
                               createMarkerColor(0.0, 0.7, 0.0, 0.5));

  addMarkers<avm::Lane>(
      gpkg_interface, &marker_array, createMarkerColor(0.2, 0.7, 0.7, 0.5),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(f.width, 0.0, 0.0); },
      "LaneArea");

  addMarkers<avm::Lane>(
      gpkg_interface, &marker_array, createMarkerColor(0.0, 0.0, 0.0, 1.0),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(0.5, 0.0, 0.0); },
      "LaneCenterLine");

  addMarkers<avm::StopLine>(
      gpkg_interface, &marker_array, createMarkerColor(1.0, 0.0, 0.0, 1.0),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(0.5, 0.0, 0.0); });

  addMarkers<avm::Crosswalk>(
      gpkg_interface, &marker_array, createMarkerColor(0.0, 1.0, 0.0, 1.0),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(f.width, 0.0, 0.0); });

  addMarkers<avm::TrafficLight>(
      gpkg_interface, &marker_array, createMarkerColor(1.0, 0.0, 1.0, 1.0),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(0.5, 0.0, 0.0); });

  return marker_array;
}

void VectorMapVisualizerNode::onBinaryGpkgMap(
    const autoware_vector_map_msgs::BinaryGpkgMap::ConstPtr& binary_gpkg_map) {
  const auto bin_data = autoware_vector_map::bridge::msg2bin(binary_gpkg_map);

  const auto gpkg_interface = std::make_unique<autoware_vector_map::io::GpkgInterface>(bin_data);

  const auto marker_array = createAllMarkers(gpkg_interface);
  pub_marker_array_.publish(marker_array);
}

VectorMapVisualizerNode::VectorMapVisualizerNode() : nh_(""), private_nh_("~") {
  pub_marker_array_ = nh_.advertise<MarkerArray>("autoware_vector_map/vizualization", 1, true);

  sub_binary_gpkg_map_ = nh_.subscribe("autoware_vector_map/binary_gpkg_map", 1,
                                       &VectorMapVisualizerNode::onBinaryGpkgMap, this);
}
