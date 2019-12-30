#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <autoware_vector_map/io/gpkg_interface.h>
#include <autoware_vector_map_msgs/BinaryGpkgMap.h>

namespace autoware_vector_map {
namespace bridge {

inline std::vector<uint8_t> msg2bin(
    const autoware_vector_map_msgs::BinaryGpkgMap::ConstPtr& bin_gpkg) {
  std::vector<uint8_t> bin_data;
  bin_data.resize(bin_gpkg->data.size());

  std::copy(std::begin(bin_gpkg->data), std::end(bin_gpkg->data), std::begin(bin_data));

  return bin_data;
}

inline autoware_vector_map_msgs::BinaryGpkgMap bin2msg(const std::vector<uint8_t>& bin_data,
                                                       const char* frame_id = "map",
                                                       const char* format_version = "",
                                                       const char* map_version = "") {
  autoware_vector_map_msgs::BinaryGpkgMap bin_gpkg{};

  bin_gpkg.header.stamp = ros::Time::now();
  bin_gpkg.header.frame_id = frame_id;

  bin_gpkg.format_version = format_version;
  bin_gpkg.map_version = map_version;

  bin_gpkg.data.clear();
  bin_gpkg.data.assign(std::begin(bin_data), std::end(bin_data));

  return bin_gpkg;
}

}  // namespace bridge
}  // namespace autoware_vector_map
