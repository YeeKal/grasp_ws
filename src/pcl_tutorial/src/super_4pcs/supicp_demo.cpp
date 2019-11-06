#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include "super4pcs_icp.h"

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


void loadPointCloud(string argv,PointCloudT::Ptr cloud){
	if(argv.substr(argv.length()-3,argv.length())=="pcd"){
		pcl::io::loadPCDFile<PointT>(argv,*cloud);
	}
	else if(argv.substr(argv.length()-3,argv.length())=="ply"){
        pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(argv, mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	}
	else{
		std::cout<<"Unknown file type."<<std::endl;
	}
}
// rosrun pcl_tutorial supicp_demo block/edit_block/del2_t.pcd imgs/pc10.pcd  -o 0.7 -d 0.01 -t 1000 -n 200 -it 20

int main (int argc,
	char* argv[])
{
	// The point clouds we will be using
	PointCloudT::Ptr cloud_temp (new PointCloudT);  // Original point cloud
	PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud

	PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
	PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

	// Checking program arguments
	if (argc < 3)
	{
		printf ("Usage :\n");
		printf ("\t\t%s file1 file2 number_of_ICP_iterations\n", argv[0]);
		PCL_ERROR ("Provide two PLY/PCD files.\n");
		return (-1);
	}


	int iterations = 1000;  // Default number of ICP iterations

	pcl::console::TicToc time;
	time.tic ();
	loadPointCloud(argv[1],cloud_in);
	loadPointCloud(argv[2],cloud_tr);
	std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;
	time.tic ();
	std::cout << "\nLoaded file " << argv[2] << " (" << cloud_icp->size () << " points) in " << time.toc () << " ms\n" << std::endl;


	// // Defining a rotation matrix and translation vector
	// Eigen::Affine3d ini_pose=Eigen::Affine3d::Identity();
	// Eigen::Matrix3d ini_rot;
    // ini_rot=Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitX());
    // ini_pose.prerotate(ini_rot);
	// Eigen::Matrix4d transformation_matrix = ini_pose.matrix();
	// pcl::transformPointCloud (*cloud_source, *cloud_in, transformation_matrix);

	pcl::Super4pcsICP<PointT,PointT> sup_icp(argc,argv);
	
	// The Iterative Closest Point algorithm
	time.tic ();
	sup_icp.setInputSource(cloud_tr);
	sup_icp.setInputTarget(cloud_in);
	//set the converge criterion to the euclidean distance
	//sup_icp.icp_.setEuclideanFitnessEpsilon(1e-06); 
	// sup_icp.icp_.setMaximumIterations (iterations);

	sup_icp.matchDebug();
	std::cout<<"Time all:"<<time.toc()<<std::endl;
	//std::cout<<"euclidean spsilon:"<<sup_icp.icp_.getEuclideanFitnessEpsilon ()<<std::endl;
	cloud_temp=sup_icp.temp_pcl_;
	cloud_icp=sup_icp.final_pcl_;


	// Visualization
	pcl::visualization::PCLVisualizer viewer ("ICP demo");
	viewer.addCoordinateSystem (1.0);
	// Create two vertically separated viewports
	int v1 (0);
	int v2 (1);
	int v3 (2);
	viewer.createViewPort (0.0, 0.0, 0.33, 1.0, v1);
	viewer.createViewPort (0.33, 0.0, 0.66, 1.0, v2);
	viewer.createViewPort (0.66, 0.0, 1.0, 1.0, v3);


	// The color we will be using
	float bckgr_gray_level = 0.0;  // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// Original point cloud is white
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
																				(int) 255 * txt_gray_lvl);
	viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
	viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);
	viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v3", v3);


	// Transformed point cloud is green
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
	viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tmp_color_h (cloud_temp, 180, 20, 20);
	viewer.addPointCloud (cloud_temp, cloud_tmp_color_h, "cloud_tmp_v2", v2);

	// ICP aligned point cloud is red
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
	viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v3", v3);

	// Adding text descriptions in each viewport
	viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText ("White: Original point cloud\nRed: Super4PCS aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);
	viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_3", v3);

	std::stringstream ss;
	ss << iterations;
	std::string iterations_cnt = "ICP iterations = " + ss.str ();
	viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt2", v2);
	viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt3", v3);

	// Set background color
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
	viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v3);


	// Set camera position and orientation
	viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize (1280, 1024);  // Visualiser window size
    viewer.addCoordinateSystem (0.5);

	// Display the visualiser
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}

	
	return (0);
}