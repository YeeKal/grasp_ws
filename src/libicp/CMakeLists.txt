cmake_minimum_required(VERSION 2.8.3)
project(libicp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set (LIBICP_SRC_DIR src)
FIND_PATH(BOOST_DIR "boost")
find_package(catkin REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES libicp
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 #include
 	${catkin_INCLUDE_DIRS}
 	${LIBICP_SRC_DIR}
	${BOOST_DIR}
)

add_library(${PROJECT_NAME}
  src/demo.cpp
  src/icp.cpp
  src/icpPointToPlane.cpp
  src/icpPointToPoint.cpp
  src/kdtree.cpp
  src/matrix.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
   ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
 )
# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

FILE(GLOB LIBICP_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# build demo program
add_executable(icp ${LIBICP_SRC_FILES})

