cmake_minimum_required(VERSION 2.8.3)
project(dagny_nav_launch)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(amcl_diff.launch)
  roslaunch_add_file_check(dagny_gmapping.launch)
  roslaunch_add_file_check(dagny_nav.launch)
endif()
