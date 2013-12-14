cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_transformer_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point_cloud_transformer_test
  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_cloud_trans src/point_cloud_trans.cpp)
target_link_libraries(point_cloud_trans ${catkin_LIBRARIES})
add_dependencies(point_cloud_trans ${catkin_EXPORTED_TARGETS}) #depend on all necessary targets
