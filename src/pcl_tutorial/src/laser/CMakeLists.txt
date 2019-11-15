# project (Super4PCS-Demos)


add_executable(read_laser read_laser.cpp)
target_link_libraries(read_laser  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(filter_laser filter_laser.cpp)
target_link_libraries(filter_laser  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(counter_pose counter_pose.cpp)
target_link_libraries(counter_pose  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

