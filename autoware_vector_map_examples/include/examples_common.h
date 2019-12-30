#pragma once

#include <bits/stdc++.h>

#include <autoware_vector_map/future/filesystem.h>

#include <ros/package.h>
#include <ros/ros.h>

namespace helper {
inline std::filesystem::path getExampleFilePath() {
  const auto package_path = ros::package::getPath("autoware_vector_map_examples");
  return std::filesystem::path(package_path) / "share" / "map" / "odaiba-sample" /
         "odaiba-sample_preprocess.gpkg";
}
}  // namespace helper
