cmake_minimum_required(VERSION 2.8.3)
project(ackermann_local_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  roscpp
  roslib
  tf
  )

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/AckermannPlanner.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_definitions(${EIGEN_DEFINITIONS})
add_library(ackermann_local_planner
  src/ackermann_planner.cpp
  src/ackermann_planner_ros.cpp
  )
add_dependencies(ackermann_local_planner
  ${ackermann_local_planner_EXPORTED_TARGETS})

# TODO: install plugin XML
# TODO: install library
