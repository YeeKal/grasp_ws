#include <iostream>
#include <string>
#include <ros/ros.h>
#include <thread>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>



using namespace std;
typedef pcl::PointXYZ PointT;


int main (int argc, char** argv)
{	
	string pcd_file="/home/yee/ros_ws/temp_ws/src/pcl_tutorial/materials/bunny.pcd";
	if(argc>1){
		pcd_file=argv[1];
	}
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cout<<"height:"<<cloud->height<<" width:"<<cloud->height<<endl;
	pcl::PointCloud<PointT>& point_cloud=*cloud;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges; //带视角的点构成的点云
	if(pcl::io::loadPCDFile<PointT>(pcd_file,*cloud)==-1){
		PCL_ERROR("could not read file\n");
		return (-1);
	}
    for(int i=0;i<cloud->points.size();i++){
        cloud->points[i].x=cloud->points[i].x/1000.0;
        cloud->points[i].y=cloud->points[i].y/1000.0;
        cloud->points[i].z=cloud->points[i].z/1000.0;
    }

    /*create point cloud*/
  
    pcl::io::savePCDFileASCII (pcd_file, *cloud);
 
    pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> sig(cloud, 0, 234, 0);
    viewer.addCoordinateSystem (1.0);
    // viewer.addPointCloud(cloud, sig, "cloud");
    viewer.addPointCloud(cloud);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}