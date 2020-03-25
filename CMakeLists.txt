cmake_minimum_required(VERSION 2.8.3)
project(zeabus_vision)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  TestMpData.msg
  TestVisionData.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

set( path_header
    "${PROJECT_SOURCE_DIR}/include"
)

set( path_dynamic
    "${PROJECT_SOURCE_DIR}/dynamic_src"
)

catkin_package(
    INCLUDE_DIRS ${path_header} ${path_dynamic}
    LIBRARIES Opencv RosSubImage
    CATKIN_DEPENDS message_runtime nav_msgs rospy sensor_msgs std_msgs roscpp
#    DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    "${path_header}" 
    "${path_dynamic}" 
)

add_subdirectory( static_src/zeabus )

