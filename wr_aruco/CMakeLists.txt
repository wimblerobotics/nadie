cmake_minimum_required(VERSION 2.8.3)
project(wr_aruco)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -Wall" )
#set(CMAKE_CXX_STANDARD 14) # use C++14

find_package(catkin REQUIRED COMPONENTS
  compressed_image_transport
  dynamic_reconfigure
  rosconsole
  roscpp
  roslaunch
  sensor_msgs
  std_msgs
  tf
)

#find_package(Boost REQUIRED COMPONENTS system thread)

#generate_dynamic_reconfigure_options( cfg/motor_controller.cfg)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  dynamic_reconfigure 
  cv_bridge
  rosconsole 
  roscpp 
  sensor_msgs 
  std_msgs 
  tf
  LIBRARIES ${PROJECT_NAME}
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
  wr_aruco_node
  src/wr_aruco_node.cpp
  src/wr_aruco.cpp
)

add_dependencies(
  wr_aruco_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  wr_aruco_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
#  boost_system boost_thread
)


## Mark executables and/or libraries for installation
#install(TARGETS motor_controller_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
