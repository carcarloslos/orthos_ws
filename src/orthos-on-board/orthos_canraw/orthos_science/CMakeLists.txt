cmake_minimum_required(VERSION 3.0.2)
project(orthos_science)

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
  ScienceCommand.srv
)

add_message_files(
  FILES
  ScienceData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 can_msgs
)

catkin_package(
   INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
   CATKIN_DEPENDS roscpp rospy std_msgs can_msgs message_runtime message_generation
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(ScienceController
        src/ScienceController.cpp
        )

set_target_properties(ScienceController PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(ScienceController PUBLIC
        include/orthos_science
        )

add_executable(orthos_science_node src/orthos_science_node.cpp)
add_executable(science_task_manager_node src/science_task_manager_node.cpp)

target_link_libraries(orthos_science_node ${catkin_LIBRARIES} ScienceController)
target_link_libraries(science_task_manager_node ${catkin_LIBRARIES})
