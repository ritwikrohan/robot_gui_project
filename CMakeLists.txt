cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs
  roscpp
  std_srvs
)

find_package(OpenCV 4 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS robotinfo_msgs roscpp geometry_msgs nav_msgs robotinfo_msgs std_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/robot_gui_class.cpp
)


add_executable(robot_gui_node
  src/robot_gui_main.cpp
)

 target_link_libraries(robot_gui_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES})
