cmake_minimum_required(VERSION 3.0.2)
project(orthos_canraw_driver)

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

add_message_files(
    FILES
    CubeMarsFeedback.msg
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
   INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
   CATKIN_DEPENDS roscpp rospy std_msgs can_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(CANRawDriver
#         src/CANRawTransmitter.cpp
#         src/CANRawReceiver.cpp
#         )

# set_target_properties(CANRawDriver PROPERTIES LINKER_LANGUAGE CXX)

# target_include_directories(CANRawDriver PUBLIC
#         include/orthos_canraw_driver
#         )

add_executable(canraw_transmitter_node src/CANRawTransmitter.cpp)
add_executable(canraw_receiver_node src/CANRawReceiver.cpp)

add_executable(joint_states_helper src/joint_states_helper.cpp)

  
target_link_libraries(canraw_transmitter_node ${catkin_LIBRARIES})
target_link_libraries(canraw_receiver_node ${catkin_LIBRARIES})

target_link_libraries(joint_states_helper ${catkin_LIBRARIES})

