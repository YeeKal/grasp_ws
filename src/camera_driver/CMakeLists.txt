cmake_minimum_required(VERSION 2.8)
project(camera_driver)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

############### dependencies ######################
# 添加cmake模块
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
#Opencv
find_package(OpenCV 3 REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIR})
#ros
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  obj_srv
  tf
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
#self file
#include_directories(../../devel/include)
include_directories(include/camera_driver)
# include_directories(include)
include_directories( ${PROJECT_SOURCE_DIR}/include )
#realsense
find_package(realsense2 REQUIRED)
include_directories(${realsense_INCLUDE_DIR})

# MUST use OpenCV in ROS !
# the original OpenCV at /usr/local/share/OpenCV do not include the Aruco modules 
# this line shoudld be default if you have successfully installed ROS, and sourced your workspace.
SET(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)
SET(realsense2_DIR /usr/lib/x86_64-linux-gnu/cmake/realsense2)

message(STATUS "########################  OpenCV library status ###############################" )
message(STATUS "                     version: ${OpenCV_VERSION}")
message(STATUS "                     include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "                     libraries: ${OpenCV_LIBS}")

message(STATUS "########################  realsense2 library status ###############################" )
message(STATUS "                     version: ${realsense_VERSION_MAJOR}")
message(STATUS "                     include path: ${realsense_INCLUDE_DIR}")
message(STATUS "                     libraries: ${realsense2_LIBRARY}")

############### execute ######################
add_executable(kinect_driver src/kinect_driver.cpp src/OpenNI2Interface.cpp)
add_executable(kinect_server src/kinect_server.cpp)
add_executable(realsense2_driver src/realsense2_driver.cpp)
add_executable(realsense2_server src/realsense2_server.cpp)

add_dependencies(realsense2_driver obj_srv_gencpp)
add_dependencies(realsense2_server obj_srv_gencpp)
add_dependencies(kinect_driver obj_srv_gencpp)
add_dependencies(kinect_server obj_srv_gencpp)


target_link_libraries(realsense2_driver
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${realsense2_LIBRARY}  
   
)
target_link_libraries(realsense2_server
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
)
target_link_libraries(kinect_driver
   ${catkin_LIBRARIES}
   ${OPENNI2_LIBRARY}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
)
target_link_libraries(kinect_server
   ${catkin_LIBRARIES}
   ${OPENNI2_LIBRARY}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
)
