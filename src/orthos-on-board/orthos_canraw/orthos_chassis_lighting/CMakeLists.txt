cmake_minimum_required(VERSION 3.0.2)
project(orthos_chassis_lighting)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  can_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  ChassisCommand.srv
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
add_library(ChassisController
        src/ChassisController.cpp
        )

set_target_properties(ChassisController PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(ChassisController PUBLIC
        include/orthos_chassis_lighting
        )

add_executable(chassis_control_node src/chassis_control_node.cpp)
  
target_link_libraries(chassis_control_node ${catkin_LIBRARIES} ChassisController)
