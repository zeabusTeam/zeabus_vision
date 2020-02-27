add_library( Opencv SHARED
    structure.cpp
    operations.cpp
)

target_link_libraries( Opencv 
    ${OpenCV_LIBRARIES} 
)
