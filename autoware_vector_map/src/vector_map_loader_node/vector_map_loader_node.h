#pragma once

#include <ros/ros.h>

class VectorMapLoaderNode {
 public:
  VectorMapLoaderNode();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher pub_binary_gpkg_map_;
};
