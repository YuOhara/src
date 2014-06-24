cmake_minimum_required(VERSION 2.8.3)
project(pr2_learn)


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

add_executable(pr2_learn src/pr2_learn.cpp)
target_link_libraries(pr2_learn ${catkin_LIBRARIES})

