#include <iostream>
#include <string>
#include <ros/ros.h>
#include <thread>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>//ply
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>


#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>



using namespace std;
typedef pcl::PointXYZ PointT;

void printHelp(std::string pro_name){
    std::cout<<"Visualize a point cloud with .obj/.ply/.pcd"<<std::endl;
    std::cout<<pro_name<<" input(.obj,.ply,pcd)"<<std::endl;
}


int main (int argc, char** argv)
{	
    printHelp(argv[0]);
    if(argc==1){
        return 0;
    }

    std::vector<int> ply_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
    std::vector<int> obj_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".obj");
    std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if(ply_file_indices.size()>0){
        pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(argv[ply_file_indices[0]], mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    }else if(obj_file_indices.size()>0){
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFile(argv[obj_file_indices[0]], mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    }else if(pcd_file_indices.size()>0){
        if(pcl::io::loadPCDFile<PointT>(argv[pcd_file_indices[0]],*cloud)==-1){
            PCL_ERROR("could not read file\n");
            return (-1);
        }
	}else{
        std::cout<<"Invalid file types.\n";
        return 0;
    }

    std::cout<<"height:"<<cloud->height<<" width:"<<cloud->width<<endl;
    std::cout<<"points:"<<cloud->points.size()<<std::endl;
    
    //view point cloud
    pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> sig(cloud, 0, 234, 0);
    viewer.addCoordinateSystem (0.3);
    // viewer.addPointCloud(cloud, sig, "cloud");
    viewer.addPointCloud(cloud);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}