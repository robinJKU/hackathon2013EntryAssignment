cmake_minimum_required(VERSION 2.8.3)
project(filter_depth_image_nodelet)

find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
	cv_bridge
	roscpp
	std_msgs
	image_transport
	nodelet
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES filter_depth_image_nodelet
  CATKIN_DEPENDS cv_bridge image_transpor roscpp sensor_msgs std_msgs nodelet
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(filter_depth_image src/filter_depth_image.cpp)
	target_link_libraries(filter_depth_image ${catkin_LIBRARIES})


