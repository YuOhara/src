cmake_minimum_required(VERSION 2.8.3)
project(point_detect)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  pcl
)

# find_package(PCL 1.2 REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_calc src/point_calc.cpp)

# target_link_libraries(point_calc ${PCL_LIBRARIES})
target_link_libraries(point_calc
   ${catkin_LIBRARIES})

