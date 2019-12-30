#include "vector_map_loader_node.h"

#include <string>

#include <autoware_vector_map/bridge/ros/message_conversion.h>
#include <autoware_vector_map/io/gpkg_interface.h>
#include <autoware_vector_map_msgs/BinaryGpkgMap.h>

using autoware_vector_map::io::GpkgInterface;
using autoware_vector_map_msgs::BinaryGpkgMap;

VectorMapLoaderNode::VectorMapLoaderNode() : nh_(""), private_nh_("~") {
  pub_binary_gpkg_map_ =
      nh_.advertise<BinaryGpkgMap>("autoware_vector_map/binary_gpkg_map", 1, true);

  std::string vector_map_path;
  private_nh_.getParam("vector_map_path", vector_map_path);

  GpkgInterface gpkg_interface(vector_map_path.c_str());

  const auto binary_gpkg_map =
      autoware_vector_map::bridge::bin2msg(gpkg_interface.toBinary(), "map", "", "");

  pub_binary_gpkg_map_.publish(binary_gpkg_map);
}
