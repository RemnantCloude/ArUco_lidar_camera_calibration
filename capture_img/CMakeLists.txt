cmake_minimum_required(VERSION 3.0.2)
project(capture_img)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENT common io)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES infrared_camera
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIB_DIRS} lib)

add_executable(cap_img  src/capture_img.cpp)
target_link_libraries(cap_img  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})