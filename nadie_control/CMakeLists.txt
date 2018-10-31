
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
#add_executable (
#  WRDifferentialDrive_node
#  src/LysanderMotor.cpp
#  src/RoboClawDriver.cpp
#  src/WRDifferentialDrive_node.cpp
#  src/WRDifferentialDrive.cpp
#)

#add_dependencies(
#  WRDifferentialDrive_node
#  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#  ${catkin_EXPORTED_TARGETS}
#)

#target_link_libraries(
#  WRDifferentialDrive_node
#  ${catkin_LIBRARIES} 
#  boost_system boost_thread
#)

#add_executable(motor_controller_node src/motorController.cpp src/motorSkidSteerDrive.cpp)

#add_dependencies(
#  motor_controller_node
#  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#  ${catkin_EXPORTED_TARGETS}
#)


## Specify libraries to link a library or executable target against
#target_link_libraries(motor_controller_node
#  ${catkin_LIBRARIES} 
#)


## Mark executables and/or libraries for installation
#install(TARGETS motor_controller_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
