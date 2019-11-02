#include <ros/ros.h>
#include <iostream>
#include <string>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> // pcd读写相关头文件
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> //kdtree
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h> //keypoint
// pcl in ros
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

typedef pcl::PointXYZ PointT;
float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

int main (int argc, char** argv)
{	
	string pcd_file="/home/yee/ros_ws/temp_ws/src/pcl_tutorial/materials/bunny.pcd";
	if(argc>0){
		pcd_file=argv[1];
	}
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>& point_cloud=*cloud;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges; //带视角的点构成的点云
	if(pcl::io::loadPCDFile<PointT>(pcd_file,*cloud)==-1){
		PCL_ERROR("could not read file\n");
		return (-1);
	}
	// setUnseenToMaxRange = true;
	// cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
	// for (float x=-0.5f; x<=0.5f; x+=0.01f)
	// {
	// 	for (float y=-0.5f; y<=0.5f; y+=0.01f)
	// 	{
	// 		PointT point;  
	// 		point.x = x;  point.y = y;  point.z = 2.0f - y;
	// 		point_cloud.points.push_back (point); //设置点云中点的坐标
	// 	}
	// }
	// point_cloud.width = (int) point_cloud.points.size ();  
	// point_cloud.height = 1;


	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                               point_cloud.sensor_origin_[1],
                                                               point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f (point_cloud.sensor_orientation_);

	//range image
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage); //创建RangeImage对象（指针）
	pcl::RangeImage& range_image = *range_image_ptr;  //引用
	range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
									scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size); //从点云创建深度图像
	range_image.integrateFarRanges (far_ranges); //整合远距离点云
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange ();
//viewer
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0.8, 0);
	viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	viewer.initCameraParameters ();

	pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  	range_image_widget.showRangeImage (range_image);

	pcl::RangeImageBorderExtractor range_image_border_extractor; //创建深度图像的边界提取器，用于提取NARF关键点
	pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor); //创建NARF对象
	narf_keypoint_detector.setRangeImage (&range_image);
	narf_keypoint_detector.getParameters ().support_size = support_size;

	pcl::PointCloud<int> keypoint_indices; //用于存储关键点的索引
	narf_keypoint_detector.compute (keypoint_indices); //计算NARF关键点
	std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

	pcl::PointCloud<PointT>::Ptr keypoints_ptr (new pcl::PointCloud<PointT>); //创建关键点指针
	pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr; //引用
	keypoints.points.resize (keypoint_indices.points.size ()); //点云变形，无序
	for (size_t i=0; i<keypoint_indices.points.size (); ++i)
		keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

  // -----Main loop-----
	while (!viewer.wasStopped ())
	{
		range_image_widget.spinOnce ();  // 处理 GUI事件
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}


	return 0;
}
