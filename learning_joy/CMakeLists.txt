cmake_minimum_required(VERSION 3.0.2)
project(learning_joy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  ackermann_msgs
)

## Declare the package dependencies
catkin_package(
  CATKIN_DEPENDS joy roscpp ackermann_msgs
)

## Specify the include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Define the executable
add_executable(ugv_joystick src/ugv_joystick.cpp)

## Link the executable with catkin libraries
target_link_libraries(ugv_joystick ${catkin_LIBRARIES})
