#include <iostream>
#include <string>
#include <ros/ros.h>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>



using namespace std;
typedef pcl::PointXYZ PointT;


int main (int argc, char** argv)
{	
	string obbj_file="/home/yee/ros_ws/temp_ws/block/block_collision_mm.obj";
	string pcd_file="/home/yee/ros_ws/temp_ws/block/block_collision_mm.pcd";
	if(argc>1){
		obbj_file=argv[1];
	}

	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileOBJ(obbj_file,mesh);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	//convert
	pcl::fromPCLPointCloud2(mesh.cloud,*cloud);
	cout<<"height:"<<cloud->height<<" width:"<<cloud->height<<endl;

	//save to pcd
	pcl::io::savePCDFileASCII(pcd_file, *cloud);


		pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sig(cloud, 0, 234, 0);
		viewer.addPointCloud(cloud, sig, "cloud");
		while (!viewer.wasStopped())
		{
				viewer.spinOnce();
		}
}