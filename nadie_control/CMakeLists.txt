
cmake_minimum_required(VERSION 2.8.3)
project(nadie_control)

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

if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  find_package(rostest REQUIRED)
  include_directories(${GTest_INCLUDE_DIRS})
endif()

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
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTest_INCLUDE_DIRS}
)

## Declare a C++ library
add_library (
  ${PROJECT_NAME}
  src/motor_controller.cpp
  src/wimble_robotics_motor_controller.cpp  
)

## Declare a C++ executable
add_executable (
  nadie_motor_controller_node
  src/motor_controller.cpp
  src/motor_controller_node.cpp
  src/wimble_robotics_motor_controller.cpp
)

add_dependencies(
  nadie_motor_controller_node
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  nadie_motor_controller_node
  ${catkin_LIBRARIES} 
  boost_system boost_thread
)

#catkin_add_gtest(${PROJECT_NAME}-utest test/M1.cpp)

#if(TARGET M1_test)
#  target_link_libraries(
#    ${PROJECT_NAME}-utest
#    ${PROJECT_NAME}
#    ${OpenCV_LIBRARIES}
#  )
#endif()

if(CATKIN_ENABLE_TESTING)
#  catkin_add_gtest(${PROJECT_NAME}-utest test/M1.cpp)
#  target_link_libraries(
#    ${PROJECT_NAME}-utest
#    ${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#  )

  add_rostest_gtest(
    ${PROJECT_NAME}-M2_test 
    test/launch/M2.test
    test/M2.cpp)
  target_link_libraries(
    ${PROJECT_NAME}-M2_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

## Mark executables and/or libraries for installation
#install(TARGETS motor_controller_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
