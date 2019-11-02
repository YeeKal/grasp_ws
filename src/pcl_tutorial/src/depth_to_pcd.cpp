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


int main(int argc, char ** argv){
        ros::init(argc, argv, "pcl_write");  
        ros::NodeHandle nh;  
        cout<< "Press space to record point cloud to a file."<<endl;  
    
        viewer = createViewer();  
    
        ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cloudCB);  
    
        ros::Rate rate(30.0);  
    
        while (ros::ok() && ! viewer->wasStopped())  
        {  
            ros::spinOnce();  
            rate.sleep();  
        }  
   
    return 0;
}