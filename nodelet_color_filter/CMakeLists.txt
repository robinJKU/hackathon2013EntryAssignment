cmake_minimum_required(VERSION 2.8.3)
project(nodelet_color_filter)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
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
  LIBRARIES ImageConverter
  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp rospy sensor_msgs std_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_executable(ImageConverter src/ImageConverter.cpp)
## Specify libraries to link a library or executable target against
 target_link_libraries(ImageConverter ${catkin_LIBRARIES} )
add_dependencies(ImageConverter ${catkin_EXPORTED_TARGETS})





