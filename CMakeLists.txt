cmake_minimum_required(VERSION 3.0.2)
project(radiation_layer)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
  radiation_msgs
)

generate_dynamic_reconfigure_options(
  cfg/RadiationLayer.cfg
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_library(rad_layer src/rad_layer.cpp)
add_dependencies(rad_layer ${catkin_EXPORTED_TARGETS})