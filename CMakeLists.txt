cmake_minimum_required(VERSION 3.0.2)
project(odometry_from_twist)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  nav_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_from_twist_node src/odometry_from_twist.cpp)
target_link_libraries(odometry_from_twist_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)