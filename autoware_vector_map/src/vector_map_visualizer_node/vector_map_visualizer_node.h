#pragma once

#include <ros/ros.h>

#include <autoware_vector_map_msgs/BinaryGpkgMap.h>

class VectorMapVisualizerNode {
 public:
  VectorMapVisualizerNode();

  void onBinaryGpkgMap(const autoware_vector_map_msgs::BinaryGpkgMap::ConstPtr& binary_gpkg_map);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_marker_array_;

  ros::Subscriber sub_binary_gpkg_map_;
};
