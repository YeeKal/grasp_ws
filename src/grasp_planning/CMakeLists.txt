cmake_minimum_required(VERSION 2.8.3)
project(grasp_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(ompl REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    moveit_simple_controller_manager
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_planners_ompl
    pluginlib
    geometric_shapes
    moveit_visual_tools
    yeebot_core
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "include/sda5f_ompl_planning"
  "/home/yee/ros_ws/yeebot/devel/include"
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# add_executable(dual_arm src/dual_arm.cpp)
# target_link_libraries(dual_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(cbirrt_demo src/cbirrt_demo.cpp)
target_link_libraries(cbirrt_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

