cmake_minimum_required(VERSION 2.8.3)
project(dubins_plus)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
  geometry_msgs
  rosunit
  tf
  )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dubins_plus
  CATKIN_DEPENDS geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a cpp library
add_library(dubins_plus src/dubins_plus.cpp)
target_link_libraries(dubins_plus ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS dubins_plus
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING) 
  catkin_add_gtest(test_dubins_plus test/dubins_plus.cpp)
  target_link_libraries(test_dubins_plus dubins_plus)
endif()
