#include <ros/ros.h>

#include "vector_map_loader_node.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "vector_map_loader");

  VectorMapLoaderNode node;

  ros::spin();

  return 0;
}
