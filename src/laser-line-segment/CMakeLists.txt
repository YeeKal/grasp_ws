cmake_minimum_required(VERSION 2.8.3)
project(laserline)

find_package(Boost REQUIRED COMPONENTS system thread)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    diagnostic_updater
    driver_base
    dynamic_reconfigure
    rosconsole
    roscpp
    self_test
    sensor_msgs
    rospy
    std_msgs
    message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS rosconsole
    LIBRARIES libhokuyo
    CATKIN_DEPENDS message_runtime
)

find_package(PkgConfig REQUIRED)

pkg_check_modules( ncurses REQUIRED ncurses)


## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(laserline src/main.cpp src/laser_feature_ros.cpp src/line_feature.cpp)

target_link_libraries(laserline ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${ncurses_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS laserline
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/laserline.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
