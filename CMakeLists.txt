cmake_minimum_required(VERSION 3.0.2)
project(odometry_from_twist)

add_compile_options(-Wall -Wextra -O2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  
  tf2_ros
  tf2_geometry_msgs
  
  geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

##################################
# catkin specific configuration ##
##################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if your package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ground_segmentation
#  CATKIN_DEPENDS pcl_ros roscpp tf2_eigen tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_node src/odometry_from_twist.cpp)
target_link_libraries(odometry_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)