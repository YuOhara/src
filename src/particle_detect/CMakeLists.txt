cmake_minimum_required(VERSION 2.8.3)
project(particle_detect)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  image_transport
)

find_package(OpenCV)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge
    LIBRARIES
    INCLUDE_DIRS
    DEPENDS OpenCV 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(particle_detect src/particle_detect.cpp)


target_link_libraries(particle_detect ${catkin_LIBRARIES})