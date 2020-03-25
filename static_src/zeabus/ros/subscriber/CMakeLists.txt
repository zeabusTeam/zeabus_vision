add_library( RosSubImage SHARED
    image.cpp )

target_link_libraries( RosSubImage
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES} )
