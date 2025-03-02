cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(hesai_ros_driver)

#=======================================
# Version information
#=======================================
set(VERSION_MAJOR 2)
set(VERSION_MINOR 0)
set(VERSION_TINY  7)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/Version.h"
)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZIRT)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
message(=============================================================)
message("-- ROS_VERSION is $ENV{ROS_VERSION}")
message(=============================================================)


#========================
# Project details / setup
#========================
set(PROJECT_NAME hesai_ros_driver)

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")

if (CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-O3)
endif()

add_definitions(-std=c++17 -Wwrite-strings)


add_compile_options(-Wwrite-strings)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -fopenmp -Wwrite-strings")
SET(CMAKE_BUILD_TYPE Release)
#========================
# Dependencies Setup
#========================

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

#ROS#
#Catkin#

find_package(roscpp 1.12 QUIET)
find_package(roslib QUIET)
find_package(rosbag REQUIRED)
# include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS} ${rosbag_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES} ${rosbag_LIBRARIES})
add_definitions(-DROS_FOUND)
add_definitions(-DRUN_IN_ROS_WORKSPACE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  image_transport
  roslib
  rosbag)

add_message_files(
  FILES
  "UdpPacket.msg"
  "UdpFrame.msg"
  "LossPacket.msg"
  "Ptp.msg"
  "Firetime.msg"
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS OpenCV
  CATKIN_DEPENDS roscpp roslib image_transport cv_bridge sensor_msgs std_msgs rosbag
  INCLUDE_DIRS src
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${roslib_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES} ${rosbag_LIBRARIES})

# # Set link libraries
list(APPEND LINK_LIBS
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

list(APPEND LINK_LIBS ${catkin_LIBRARIES})

#Others#
find_package(yaml-cpp REQUIRED)

#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)

#Driver core#
add_subdirectory(${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk)

#========================
# Build Setup
#========================
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(
  ${Boost_INCLUDE_DIRS}
  )
link_directories(
  ${Boost_LIBRARY_DIRS} 
  ${OpenCV_LIBRARY_DIRS}
  )

  message(=============================================================)
  message("-- CUDA support disabled by GrandTour team.")
  message(=============================================================)
  add_executable(hesai_ros_driver_node
                node/hesai_ros_driver_node.cc
                src/manager/node_manager.cc
                )
  target_link_libraries(hesai_ros_driver_node
              ${catkin_LIBRARIES}                
              ${YAML_CPP_LIBRARIES}
              ${Boost_LIBRARIES}
              ${OpenCV_LIBRARIES}
              ${Eigen_LIBRARIES}
              source_lib
              container_lib
              ptcClient_lib
              ptcParser_lib
              log_lib
              # libhesai
  )            

target_compile_options(hesai_ros_driver_node PRIVATE -Wwrite-strings)

target_include_directories(hesai_ros_driver_node PRIVATE
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/Lidar
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/UdpParser
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/UdpParser/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/UdpParser/src
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/UdpProtocol
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/Source/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/Container/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/Container/src
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/UdpParserGpu
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/UdpParserGpu/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/UdpParserGpu/src
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/PtcClient/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/PtcParser/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/PtcParser
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/Logger/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/libhesai/include
  ${PROJECT_SOURCE_DIR}/src/driver/hesai_lidar_sdk/driver
  src/manager
  src/msg/ros_msg
  src/msg/rs_msg
  src/utility
  ${CMAKE_CURRENT_BINARY_DIR}
)  

#Ros#
target_link_libraries(hesai_ros_driver_node  ${ROS_LIBS})
install(TARGETS hesai_ros_driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})



