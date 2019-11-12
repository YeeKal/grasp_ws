# project (Super4PCS-Demos)


add_executable(read_laser read_laser.cpp)
target_link_libraries(read_laser  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

