add_library( Opencv
    structure.cpp
    operations.cpp
    convert.cpp
)

target_link_libraries( Opencv 
    ${OpenCV_LIBRARIES} 
)
