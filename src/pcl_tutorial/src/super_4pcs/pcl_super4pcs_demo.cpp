#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <super4pcs/shared4pcs.h>

#include "pcl_super4pcs.h"
#include "demo-utils.h"

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

using namespace GlobalRegistration;

bool loadPointCloud(std::string argv,PointCloudT::Ptr cloud){
	if(argv.substr(argv.length()-3,argv.length())=="pcd"){
		if(pcl::io::loadPCDFile<PointNT> (argv, *cloud)>=0){
			return true;
		}
	}
	else if(argv.substr(argv.length()-3,argv.length())=="ply"){
		if(pcl::io::loadOBJFile<PointNT> (argv, *cloud)>=0){
			return true;
		}
	}
	else if(argv.substr(argv.length()-3,argv.length())=="obj"){
		if(pcl::io::loadOBJFile<PointNT> (argv, *cloud)>=0)
		{
			return true;
		}
	}
	else{
		std::cout<<"Unknown file type."<<std::endl;
		return false;
	}

	std::cout<<"Error loading file:"<<argv<<"\n";
	return false;
}

void preProcess(PointCloudT::Ptr cloud){
	// std::vector<int> indices; 
  	// pcl::removeNaNFromPointCloud(*scene,*scene, indices); 
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
	nest.setRadiusSearch (0.01);
	nest.setInputCloud (cloud);
	nest.compute (*cloud);
}

void printHelp(){
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "Match two point cloud(PLY/PCD/OBJ) by Super4PCS."<<std::endl;
    std::cout << "Usage:  <input1_file> <input2_file> "<<std::endl;
    std::cout << "Example:  rosrun pcl_tutorial pcl_super4pcs_demo block/edit_block/del2.pcd imgs/pc5.pcd  -o 0.7 -d 0.01 -t 1000 -n 200 " << std::endl;
    std::cout << "Notice: -h to show the detail parameters" << std::endl;
    std::cout << "***************************************************************************" << std::endl;

}

// Align a rigid object to a scene with clutter and occlusions
// rosrun pcl_tutorial pcl_super4pcs_demo /home/yee/program/Super4PCS/assets/hippo1.obj  /home/yee/program/Super4PCS/assets/hippo2.obj -o 0.7 -d 0.01 -t 1000 -n 200

int main (int argc, char **argv)
{
	printHelp();
// Point clouds
PointCloudT::Ptr object (new PointCloudT);
PointCloudT::Ptr object_aligned (new PointCloudT);
PointCloudT::Ptr scene (new PointCloudT);

// Get input object and scene
if (argc < 4)
{
	pcl::console::print_error ("Syntax is: %s scene.obj object.obj [PARAMS]\n", argv[0]);
	Demo::printParameterList();
	return (-1);
}

// Load object and scene
pcl::console::print_highlight ("Loading point clouds...\n");
if (!loadPointCloud(argv[1],object) || !loadPointCloud(argv[2],scene))
{
	pcl::console::print_error ("Error loading object/scene file!\n");
	return (-1);
}

std::cout<<"Estimating the point clouds normals."<<std::endl;
// preProcess(object);
// preProcess(scene);

// Load Super4pcs parameters
Demo::getArgs(argc, argv);

pcl::Super4PCS<PointNT,PointNT> align;
Demo::setOptionsFromArgs(align.options_);

// Downsample
//  pcl::console::print_highlight ("Downsampling...\n");
//  pcl::VoxelGrid<PointNT> grid;
//  const float leaf = 0.005f;r
//  grid.setLeafSize (leaf, leaf, leaf);
//  grid.setInputCloud (object);
//  grid.filter (*object);
//  grid.setInputCloud (scene);
//  grid.filter (*scene);

// Perform alignment
pcl::console::print_highlight ("Starting alignment...\n");
align.setInputSource (object);
align.setInputTarget (scene);

{
	pcl::ScopeTime t("Alignment");
	align.align (*object_aligned);
}

if (align.hasConverged ())
{
	// Print results
	printf ("\n");
	Eigen::Matrix4f transformation = align.getFinalTransformation ();
	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
	pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
	pcl::console::print_info ("\n");
	pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
	pcl::console::print_info ("\n");

	// Show alignment
	pcl::visualization::PCLVisualizer visu("Alignment - Super4PCS");
	visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
	visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
	visu.spin ();
}
else
{
	pcl::console::print_error ("Alignment failed!\n");
	return (-1);
}

return (0);
}
