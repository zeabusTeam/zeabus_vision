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

set( zeabus_vision_include_path
    "${PROJECT_SOURCE_DIR}/include"
)

catkin_package(
    INCLUDE_DIRS ${zeabus_vision_include_path}
    LIBRARIES Opencv
    CATKIN_DEPENDS message_runtime nav_msgs rospy sensor_msgs std_msgs roscpp
#    DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    "${zeabus_vision_include_path}"  
)

add_subdirectory( static_src/zeabus )
