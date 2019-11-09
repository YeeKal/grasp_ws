#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <obj_srv/rgbd_image.h>

using namespace std;
using namespace cv;

sensor_msgs::Image rgb_image;
sensor_msgs::Image depth_image;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_rgb;
  image_transport::Subscriber image_sub_depth;
  
public:

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    // image_sub_rgb = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb_rgb, this);///realsense_sr300/rgb ///camera/rgb/image_rect_color
    // image_sub_depth = it_.subscribe("/camera/depth_registered/image_raw", 1, &ImageConverter::imageCb_depth, this);///realsense_sr300/depth ///camera/depth_registered/image_raw

    image_sub_rgb = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb_rgb, this);///realsense_sr300/rgb ///camera/rgb/image_rect_color
    image_sub_depth = it_.subscribe("/camera/depth/image_rect_raw", 1, &ImageConverter::imageCb_depth, this);///realsense_sr300/depth ///camera/depth_registered/image_raw
  }

  ~ImageConverter()
  {
  }

  void imageCb_rgb(const sensor_msgs::ImageConstPtr& msg)
  {
     rgb_image = *msg;
  }
  void imageCb_depth(const sensor_msgs::ImageConstPtr& msg)
  {  
     depth_image = *msg;
  }

};

bool get_image(obj_srv::rgbd_image::Request &req,obj_srv::rgbd_image::Response &res)
{
    if (req.start) 
    {
         res.rgb_image = rgb_image;
         res.depth_image = depth_image;
    }
    ROS_INFO("success");
    return true;
}


int main(int argc, char** argv)
{    
   
  ros::init(argc, argv, "get_image");
  ImageConverter ic; 
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("get_image", get_image);

  ros::Rate loop_rate(200);
  while (ros::ok())
  {

     ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}

