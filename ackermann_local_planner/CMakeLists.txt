cmake_minimum_required(VERSION 2.8.3)
project(ackermann_local_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  costmap_2d
  dubins_plus
  dynamic_reconfigure
  nav_core
  nav_msgs
  roscpp
  roslib
  tf
  )

generate_dynamic_reconfigure_options(
  cfg/AckermannPlanner.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(ackermann_local_planner
  src/ackermann_planner_ros.cpp
  )
add_dependencies(ackermann_local_planner
  ${ackermann_local_planner_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(ackermann_local_planner 
  ${catkin_LIBRARIES}
  )

install(TARGETS ackermann_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(FILES blp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
