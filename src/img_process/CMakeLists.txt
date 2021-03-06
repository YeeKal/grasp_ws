cmake_minimum_required(VERSION 2.8.3)
project(img_process)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

set(OpenNI2_LIBRARY 
    /usr/lib/libOpenNI2.so
    /usr/lib/OpenNI2/Drivers/libDummyDevice.so
    /usr/lib/OpenNI2/Drivers/libOniFile.so
    /usr/lib/OpenNI2/Drivers/libPS1080.so
    /usr/lib/OpenNI2/Drivers/libPSLink.so
    )


find_package( OpenCV  REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  sensor_msgs

  pcl_tutorial
)


catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(hsv_seg src/hsv_seg.cpp)
target_link_libraries(hsv_seg ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(average_hsv src/average_hsv.cpp)
target_link_libraries(average_hsv ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(view_hsv src/view_hsv.cpp)
target_link_libraries(view_hsv ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(openni2_demo src/openni2_demo.cpp)
target_link_libraries(openni2_demo ${catkin_LIBRARIES} ${OpenCV_LIBS} ${OpenNI2_LIBRARY})


