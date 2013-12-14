cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_transformer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PC2TransformerNodelet
  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs tf nodelet
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Create the nodelet tutorial library
add_library(PC2TransformerNodelet src/point_cloud_transform.cpp) #nodelet_PC2_transformer

target_link_libraries(PC2TransformerNodelet 
	${catkin_LIBRARIES}
)

#if(catkin_EXPORTED_LIBRARIES)
#  add_dependencies(PC2TransformerNodelet ) # ${catkin_EXPORTED_LIBRARIES})
#endif()

install(TARGETS PC2TransformerNodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_PC2_transformer.xml point_cloud_transformer.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
