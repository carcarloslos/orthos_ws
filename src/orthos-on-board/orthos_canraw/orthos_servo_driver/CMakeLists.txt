cmake_minimum_required(VERSION 3.0.2)
project(orthos_servo_driver)

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
  tf
)

add_service_files(
  FILES
  ServoCommand.srv
)

add_message_files(
  FILES
  ServoControllerFeedback.msg
  CubeMarsFeedback.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 can_msgs
)

catkin_package(
   INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
   CATKIN_DEPENDS roscpp rospy std_msgs can_msgs tf message_runtime message_generation
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(ServoController
        src/ServoController.cpp
        )

set_target_properties(ServoController PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(ServoController PUBLIC
        include/orthos_servo_driver
        )

add_executable(wrist_control_node src/wrist_control_node.cpp)
add_executable(swivel_control_node src/swivel_control_node.cpp)
add_executable(swivel_tf_node src/swivel_tf_node.cpp)

target_link_libraries(wrist_control_node ${catkin_LIBRARIES} ServoController)
target_link_libraries(swivel_control_node ${catkin_LIBRARIES} ServoController)
target_link_libraries(swivel_tf_node ${catkin_LIBRARIES} ServoController)
