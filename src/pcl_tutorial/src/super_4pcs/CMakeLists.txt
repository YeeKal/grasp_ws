# project (Super4PCS-Demos)

# add_subdirectory(Super4PCS)
include("/usr/local/lib/cmake/Super4PCSConfig.cmake")

# openmp
find_package(OpenMP)
if(OPENMP_FOUND)
    message(STATUS "Enable OpenMP")
    add_definitions("-DSUPER4PCS_USE_OPENMP -DEIGEN_DONT_PARALLELIZE")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

SET(Super4PCS_INCLUDE_DIR /usr/local/include)
SET(Super4PCS_LIB_DIR /usr/local/lib)
SET(Super4PCS_LIBRARIES super4pcs_accel super4pcs_io super4pcs_algo)

message (STATUS "Super4PCS_INCLUDE_DIR: ${Super4PCS_INCLUDE_DIR}")
message (STATUS "Super4PCS_LIB_DIR    : ${Super4PCS_LIB_DIR}")
message (STATUS "Super4PCS_LIBRARIES  : ${Super4PCS_LIBRARIES}")

# super4pcs
#find_package(Super4PCS REQUIRED)
include_directories(${Super4PCS_INCLUDE_DIR})
link_directories(${Super4PCS_LIB_DIR})

add_executable(super4pcs_demo super4pcs_demo.cpp)
target_link_libraries(super4pcs_demo  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Super4PCS_LIBRARIES})

add_executable(pcl_super4pcs_demo pcl_super4pcs_demo.cpp)
target_link_libraries(pcl_super4pcs_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Super4PCS_LIBRARIES})

