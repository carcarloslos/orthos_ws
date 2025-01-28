cmake_minimum_required(VERSION 3.0.2)
project(orthos_cubemars_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  can_msgs
  message_generation
)

# Declare a custom message file
add_message_files(
  FILES
  CubeMarsPosVelGoal.msg 
  CubeMarsFeedback.msg
  CubeMarsCurrentLoopMsg.msg
)

add_service_files(
  FILES
  SetOrigin.srv
  SetPositionGoal.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 can_msgs
)

catkin_package(
   INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
   CATKIN_DEPENDS roscpp rospy std_msgs can_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# Declare a C++ library
add_library(CubeMarsMotor
        src/CubeMarsMotor.cpp
        )

set_target_properties(CubeMarsMotor PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(CubeMarsMotor PUBLIC
        include/orthos_cubemars_driver
        )

add_executable(cubemars_driver_node src/CubeMarsDriver.cpp)
  
target_link_libraries(cubemars_driver_node ${catkin_LIBRARIES} CubeMarsMotor)


