cmake_minimum_required(VERSION 2.8.3)
project(ackermann_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  dynamic_reconfigure
  nav_core
  )

generate_dynamic_reconfigure_options(
  cfg/AckermannPlanner.cfg
)

add_library(ackermann_local_planner src/ackermann_planner.cpp src/ackermann_planner_ros.cpp)
