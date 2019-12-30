cmake_minimum_required(VERSION 2.8.3)
project(autoware_vector_map)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_vector_map_msgs
  geometry_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(GDAL REQUIRED)
find_package(fmt REQUIRED)

# To be replaced by std::filesystem in C++17
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    autoware_vector_map
  CATKIN_DEPENDS
    autoware_vector_map_msgs
  DEPENDS
    Boost
    EIGEN3
    GDAL
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GDAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Target
## autoware_vector_map
file(GLOB_RECURSE autoware_vector_map_FILES ${PROJECT_SOURCE_DIR}/src/autoware_vector_map/*)
add_library(autoware_vector_map
  ${autoware_vector_map_FILES}
)

target_link_libraries(autoware_vector_map
  ${Boost_LIBRARIES}
  ${GDAL_LIBRARIES}
  fmt
  ${catkin_LIBRARIES}
)

add_dependencies(autoware_vector_map
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## vector_map_loader_node
add_executable(vector_map_loader
  src/vector_map_loader_node/main.cpp
  src/vector_map_loader_node/vector_map_loader_node.cpp
)

target_link_libraries(vector_map_loader
  autoware_vector_map
  ${catkin_LIBRARIES}
)

add_dependencies(vector_map_loader
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## vector_map_visualizer_node
add_executable(vector_map_visualizer
  src/vector_map_visualizer_node/main.cpp
  src/vector_map_visualizer_node/vector_map_visualizer_node.cpp
)

target_link_libraries(vector_map_visualizer
  autoware_vector_map
  ${catkin_LIBRARIES}
)

add_dependencies(vector_map_visualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install
## executables and libraries
install(
  TARGETS
    autoware_vector_map
    vector_map_loader
    vector_map_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_autoware_vector_map
    test/test_autoware_vector_map.test
    test/src/test_autoware_vector_map.cpp
  )

  target_link_libraries(test_autoware_vector_map
    autoware_vector_map
    ${catkin_LIBRARIES}
  )
endif ()
