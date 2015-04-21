cmake_minimum_required(VERSION 2.8.3)
project(percept_generators)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  object.msg
  objectArray.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES percept_generators
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs#roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(colored_obj_extractor src/colored_obj_extractor.cpp)
target_link_libraries(colored_obj_extractor
   ${catkin_LIBRARIES}
	${PCL_LIBRARIES}
 )
