cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

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

  obj_srv
)


catkin_package(

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# tutorial
add_executable(tutorial_icp src/tutorial_icp.cpp)
target_link_libraries(tutorial_icp ${catkin_LIBRARIES})

add_executable(pcl_temp src/pcl_temp.cpp)
target_link_libraries(pcl_temp ${catkin_LIBRARIES})

add_executable(pcl_vis src/pcl_vis.cpp)
target_link_libraries(pcl_vis ${catkin_LIBRARIES})



add_executable(ply_to_pcd src/ply_to_pcd.cpp)
target_link_libraries(ply_to_pcd ${catkin_LIBRARIES})

add_executable(correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries(correspondence_grouping ${catkin_LIBRARIES})

add_executable(cylinder_segmentation src/cylinder_segmentation.cpp)
target_link_libraries(cylinder_segmentation ${catkin_LIBRARIES})

add_executable(border_extraction src/border_extraction.cpp)
target_link_libraries(border_extraction ${catkin_LIBRARIES})

add_executable(implicit_shape_model src/implicit_shape_model.cpp)
target_link_libraries(implicit_shape_model ${catkin_LIBRARIES})

add_executable(correspondence_block src/correspondence_block.cpp)
target_link_libraries(correspondence_block ${catkin_LIBRARIES})

add_executable(read_obj src/read_obj.cpp)
target_link_libraries(read_obj ${catkin_LIBRARIES})

add_executable(region_growing_rgb_segmentation src/region_growing_rgb_segmentation.cpp)
target_link_libraries(region_growing_rgb_segmentation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(alignment_prerejective_block src/alignment_prerejective_block.cpp)
target_link_libraries(alignment_prerejective_block ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mesh_sample src/mesh_sample.cpp)
target_link_libraries(mesh_sample ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(pcd_size src/pcd_size.cpp)
target_link_libraries(pcd_size ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(alignment_prerejective src/alignment_prerejective.cpp)
target_link_libraries(alignment_prerejective ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(icp_block src/icp_block.cpp)
target_link_libraries(icp_block ${catkin_LIBRARIES})


#add_executable(global_hypothesis src/global_hypothesis.cpp)
#target_link_libraries(global_hypothesis ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# add_executable(template_alignment src/template_alignment.cpp)
# target_link_libraries(implicit_shape_model ${catkin_LIBRARIES})

# filter
add_executable(passthrough src/filter/passthrough.cpp)
target_link_libraries(passthrough ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(extract_plane src/filter/extract_plane.cpp)
target_link_libraries(extract_plane ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# linemod
#add_executable(linemod_train src/linemod/linemod_train.cpp)
#target_link_libraries(linemod_train ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_executable(linemod_detect src/linemod/linemod_detect.cpp)
#target_link_libraries(linemod_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(save_depth src/save_depth.cpp)
target_link_libraries(save_depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(save_rgb src/save_rgb.cpp)
target_link_libraries(save_rgb ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(save_depth_with_mask src/save_depth_with_mask.cpp)
target_link_libraries(save_depth_with_mask ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_subdirectory(src/super_4pcs)




