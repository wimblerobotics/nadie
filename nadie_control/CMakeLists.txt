
cmake_minimum_required(VERSION 2.8.3)
project(nadie_controller)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set(ROS_BUILD_TYPE Debug)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  message_generation
  nav_msgs
  rosconsole
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  urdf
)

find_package(Boost REQUIRED COMPONENTS system thread)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RoboClawStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

#generate_dynamic_reconfigure_options( cfg/motor_controller.cfg)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_runtime rosconsole roscpp rospy sensor_msgs std_msgs tf urdf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable (
  nadie_motor_controller_node
#  src/RoboClawDriver.cpp
  src/motor_controller.cpp
  src/motor_controller_node.cpp
  src/wimble_robotics_motor_controller.cpp
)

add_dependencies(
  nadie_motor_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  nadie_motor_controller_node
  ${catkin_LIBRARIES} 
  boost_system boost_thread
)



## Mark executables and/or libraries for installation
#install(TARGETS motor_controller_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
