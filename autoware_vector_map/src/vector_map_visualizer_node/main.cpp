#include <ros/ros.h>

#include "vector_map_visualizer_node.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "vector_map_visualizer");

  VectorMapVisualizerNode node;

  ros::spin();

  return 0;
}
