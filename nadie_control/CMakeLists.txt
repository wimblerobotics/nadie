
cmake_minimum_required(VERSION 2.8.3)
project(nadie_control)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_CXX_FLAGS "-g")

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
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  transmission_interface
  urdf
)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package( PCL REQUIRED )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArduinoSensors.msg
  RoboClawStatus.msg
)

add_service_files(
  FILES
  ResetEncoders.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  dynamic_reconfigure 
  geometry_msgs 
  message_runtime 
  pcl_ros
  rosconsole 
  roscpp 
  rospy 
  sensor_msgs 
  std_msgs 
  tf
  transmission_interface
  urdf
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library (
  ${PROJECT_NAME}
  src/motor_controller.cpp
  src/nadie_motor_controller.cpp  
)

add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_generate_messages_cpp
)

## Declare a C++ executable
add_executable(
  closest_obj_node
  src/closest_obj.cpp
)

add_dependencies(
  closest_obj_node
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  closest_obj_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable (
  calibrate_node
  src/calibrate_node.cpp
  src/calibrate.cpp
)

add_dependencies(
  calibrate_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  calibrate_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
)

add_executable (
  nadie_motor_controller_node
  # src/motor_controller.cpp
  src/motor_controller_node.cpp
  #src/nadie_motor_controller.cpp
)

add_dependencies(
  nadie_motor_controller_node
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  nadie_motor_controller_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  boost_system boost_thread
)
