#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    string file1="block/block_collision_mm_midpoint20.pcd";
    string file2="block/block_collision_mm_midpoint20_downsampled.pcd";

    // Fill in the cloud data
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file1,*cloud)==-1){
		PCL_ERROR("could not read file\n");
		return (-1);
	}

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        << " data points (" <<cloud->points.size() << ").";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
        << " data points (" <<cloud_filtered->points.size() << ").";

    pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sig(cloud_filtered, 0, 234, 0);
    viewer.addPointCloud(cloud_filtered, sig, "cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return (0);
}