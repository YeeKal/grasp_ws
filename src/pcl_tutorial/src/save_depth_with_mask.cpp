
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h>  
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>  

#include <thread>
#include <pthread.h>
#include "pcl_tutorial/hsv_segmentation.h"

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace pcl; 

const double camera_cx = 319.5;//325.5//319.5 310.95 310.95
const double camera_cy = 239.5;//253.5//239.5 234.74 234.74
const double camera_fx = 570.3422;//518.0//570.3422(openni2) 615.377
const double camera_fy = 570.3422;//519.0//570.3422(openni2) 615.377
pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
boost::shared_ptr<visualization::CloudViewer> viewer;  
bool saveCloud(false);  

class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_depth;

    sensor_msgs::Image depth_image_;

    std::string topic_name_;
    std::string depth_topic_name_;
    std::string saved_folder_;
    bool save_img_;
    int filenum_;
    HSVSegmentation hsv_;
    Mat depth;

    ImageConverter(std::string rgb_topic,std::string depth_topic,std::string saved_folder="")
    :topic_name_(rgb_topic),
     depth_topic_name_(depth_topic),
     saved_folder_(saved_folder),
     hsv_(HSVSegmentation(100,123, 150,255,85,220)),
     it_(nh_),save_img_(false),filenum_(0)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe(topic_name_, 1,
        &ImageConverter::imageCb, this);
        image_sub_depth = it_.subscribe(depth_topic_name_, 1, &ImageConverter::imageCb_depth, this);///realsense_sr300/depth ///camera/depth_registered/image_raw
    }

    ~ImageConverter()
    {
    }

    void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
    {  
        depth_image_ = *msg;
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(depth_image_.height==0) return ;
        depth = cv_bridge::toCvCopy(depth_image_, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

        hsv_.setSrc(cv_ptr->image);
        hsv_.getDepthDst(depth);

        //dpth to point cloud
        scene->points.clear();
        scene_rgb->points.clear();
        for (int r=0;r<depth.rows;r++)
        {
            for (int c=0;c<depth.cols;c++)
            {
                pcl::PointXYZ p;
                pcl::PointXYZRGBA p_rgb;
                if(!(depth.at<float>(r,c)>0&&depth.at<float>(r,c)<1.3)){continue;}
                double scene_z = double(depth.at<float>(r,c));
                double scene_x = (c - camera_cx) * scene_z / camera_fx;
                double scene_y = (r - camera_cy) * scene_z / camera_fy;
                p.x = scene_x;
                p.y = scene_y;
                p.z = scene_z;
                p_rgb.x = scene_x;
                p_rgb.y = scene_y;
                p_rgb.z = scene_z;
                // p_rgb.r = rgb.ptr<uchar>(r)[c*3];
                // p_rgb.g = rgb.ptr<uchar>(r)[c*3+1];
                // p_rgb.b = rgb.ptr<uchar>(r)[c*3+2];
                p_rgb.r = cv_ptr->image.ptr<uchar>(r)[c*3+2];
                p_rgb.g = cv_ptr->image.ptr<uchar>(r)[c*3+1];
                p_rgb.b = cv_ptr->image.ptr<uchar>(r)[c*3];
                scene->points.push_back(p);
                scene_rgb->points.push_back(p_rgb);
            }
        }

        // imshow(topic_name_,cv_ptr->image);
        // imshow(depth_topic_name_,depth);

        if(! viewer->wasStopped()) viewer->showCloud(scene_rgb->makeShared());


        if(saveCloud){
            //file name
            //save rgb
            // std::stringstream stream;  
            // if(saved_folder_==""){
            //     stream << "image"<< filenum_<< ".jpg";  
            // }else{
            //     stream <<saved_folder_<< "/image"<< filenum_<< ".jpg";  
            // }
            // filenum_++;
            // std::string filename = stream.str(); 
            // cv::imwrite(filename,cv_ptr->image);
            // saveCloud=false;
            // std::cout<<"Saved "<<filenum_<<" image.\n";

            //save point cloud
            std::stringstream stream;  
            if(saved_folder_==""){
                stream << "pc"<< filenum_<< ".pcd";  
            }else{
                stream <<saved_folder_<< "/pc"<< filenum_<< ".pcd";  
            }
            std::string filename = stream.str(); 

             if(io::savePCDFile(filename, *scene_rgb, true) == 0)  
            {  
                filenum_++;  
                cout << filenum_<<" Saved."<<endl;  
            }  
            else PCL_ERROR("Problem saving %s.\n", filename.c_str());  
            saveCloud=false;
        }

        // if(cv::waitKey(10)==32){
        //     save_img_=true;
        // }
        

        // if(save_img_){
        //     //file name
        //     std::stringstream stream;  
        //     if(saved_folder_==""){
        //         stream << "image"<< filenum_<< ".jpg";  
        //     }else{
        //         stream <<saved_folder_<< "/image"<< filenum_<< ".jpg";  
        //     }
        //     filenum_++;
        //     std::string filename = stream.str(); 
        //     cv::imwrite(filename,cv_ptr->image);
        //     save_img_=false;
        //     std::cout<<"Saved "<<filenum_<<" image.\n";
        // }
    }
};

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
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "Save depth image to point cloud(.pcd) filtered by rgb image."<<std::endl;
    std::cout << "Usage:  <rgb_topic> <depth_topic> <saved folder> "<<std::endl;
    std::cout << "Example:  rosrun pcl_tutorial saverk_depth_with_mask /camera/rgb/image_rect_color /camera/depth_registered/image_raw imgs" << std::endl;
    std::cout << "Notice: Press 'space' to save one file." << std::endl;
    std::cout << "***************************************************************************" << std::endl;

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    printHelp();

    std::string rgb_topic="/camera/rgb/image_rect_color";
    std::string depth_topic="/camera/depth_registered/image_raw";
    std::string saved_folder="";
    ////camera/rgb/image_rect_color
    ///camera/depth_registered/image_raw
    if(argc==2) std::cout<<"Not valid args.\n";
    if(argc>2){
        rgb_topic=argv[1];
        depth_topic=argv[2];
    }
    if(argc>3){
        saved_folder=argv[3];
    }
    viewer = createViewer();
    ImageConverter ic(rgb_topic,depth_topic, saved_folder);
    
    ros::Rate rate(30.0);  
    
    while (ros::ok() && ! viewer->wasStopped())  
    {  
        ros::spinOnce();  
        rate.sleep();  
    } 
    return 0;
}