#include <ros/ros.h>
#include <iostream>
#include <string>
#include <thread>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> // pcd读写相关头文件
#include <pcl/io/ply_io.h>//ply
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> //kdtree
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h> //keypoint

using namespace std;
typedef pcl::PointXYZRGBA PointType;

int main(int argc,char **argv){
    string file="/home/yee/ros_ws/temp_ws/src/pcl_tutorial/materials/bunny/reconstruction/bun_zipper.ply";
    if(argc>1){
        file=argv[1];
    }
    string file_save="/home/yee/ros_ws/temp_ws/src/pcl_tutorial/materials/temp.pcd";
    if(argc>2){
        file_save=argv[2];
    }


    pcl::PolygonMesh mesh;
	pcl::io::loadPLYFile(file, mesh);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("polygon"));
	viewer->addPolygonMesh(mesh,"mesh");
	// while(!viewer->wasStopped()){
	// 	viewer->spinOnce();
	// }


    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    fromPCLPointCloud2(mesh.cloud, *cloud);
    for(int i=0;i<cloud->points.size();i++){
        cloud->points[i].x=cloud->points[i].x/1000;
        cloud->points[i].y=cloud->points[i].y/1000;
        cloud->points[i].z=cloud->points[i].z/1000;

    }
    pcl::io::savePCDFile(file_save, *cloud);

    // pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    // if (pcl::io::loadPLYFile<PointType>(file, *cloud) == -1) //* load the file 
    // {
    //     PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    //     system("PAUSE");
    //     return (-1);
    // }
    // pcl::io::savePCDFile("bunny_zipper.pcd", *cloud );
    // cout<<"ahah\n";

    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointType>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
    return 0;

}