#include <iostream>  
#include <ros/ros.h>  
#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/io/pcd_io.h>  
  
#include <pcl/visualization/cloud_viewer.h>  
  
#include <sensor_msgs/PointCloud2.h>  

using namespace std;
//using namespace cv;

using namespace pcl;  
  
unsigned int filesNum = 0;  
bool saveCloud(false);  

boost::shared_ptr<visualization::CloudViewer> viewer;  
  
void cloudCB(const sensor_msgs::PointCloud2& input)  
{  
    pcl::PointCloud<pcl::PointXYZRGBA> cloud; // With color  
  
    pcl::fromROSMsg(input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>  
  
    if(! viewer->wasStopped()) viewer->showCloud(cloud.makeShared());  
  
    if(saveCloud)  
    {  
        stringstream stream;  
        stream << "inputCloud"<< filesNum<< ".pcd";  
        string filename = stream.str();  
  
        if(io::savePCDFile(filename, cloud, true) == 0)  
        {  
            filesNum++;  
            cout << filename<<" Saved."<<endl;  
        }  
        else PCL_ERROR("Problem saving %s.\n", filename.c_str());  
  
        saveCloud = false;  
  
    }  
  
  
}  

void  
keyboardEventOccured(const visualization::KeyboardEvent& event, void* nothing)  
{  
    if(event.getKeySym() == "space"&& event.keyDown())  
        saveCloud = true;  
  
}  

// Creates, initializes and returns a new viewer.  
boost::shared_ptr<visualization::CloudViewer> createViewer()  
{  
    boost::shared_ptr<visualization::CloudViewer> v(new visualization::CloudViewer("OpenNI viewer"));  
    v->registerKeyboardCallback(keyboardEventOccured);  
  
    return(v);  
}  
void printHelp(){
    std::cout << "Save depth image to point cloud(.pcd)."<<std::endl;
    std::cout << "Usage:  <rgb_topic> <depth_topic> <saved folder>"<<std::endl;
    std::cout << "Example:  rosrun pcl_tutorial saverk_depth_with_mask /camera/rgb/image_rect_color /camera/depth_registered/image_raw imgs" << std::endl;
    std::cout << "Notice: Press space to record point cloud to a pcd file." << std::endl;

}

int main(int argc, char ** argv){
        ros::init(argc, argv, "save_depth");  
        ros::NodeHandle nh;  
        cout<< "Press space to record point cloud to a pcd file."<<endl;  
        string topic_name="/camera/depth/color/points";//"/camera/depth_registered/points";
        if(argc>1){
            topic_name=argv[1];
        }
    
        viewer = createViewer();  

        ros::Subscriber pcl_sub = nh.subscribe(topic_name, 1, cloudCB);  
    
        ros::Rate rate(30.0);  
    
        while (ros::ok() && ! viewer->wasStopped())  
        {  
            ros::spinOnce();  
            rate.sleep();  
        }  
   
    return 0;
}