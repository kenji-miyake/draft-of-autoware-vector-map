cmake_minimum_required(VERSION 2.8.3)
project(autoware_vector_map_examples)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_vector_map
  roscpp
  roslib
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_vector_map
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Target
add_executable(example1 src/example1/main.cpp)
target_link_libraries(example1 ${catkin_LIBRARIES})

add_executable(example2 src/example2/main.cpp)
target_link_libraries(example2 ${catkin_LIBRARIES})

# Install
install(
  TARGETS
    example1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
