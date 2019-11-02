
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <pthread.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    std::string topic_name_;
    std::string saved_folder_;
    bool save_img_;
    int filenum_;


    ImageConverter(std::string topic="/camera/color/image_raw",std::string saved_folder="")
    : topic_name_(topic),saved_folder_(saved_folder),it_(nh_),save_img_(false),filenum_(0)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe(topic_name_, 1,
        &ImageConverter::imageCb, this);

        cv::namedWindow(topic_name_);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(topic_name_);
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

        imshow(topic_name_,cv_ptr->image);
        if(cv::waitKey(5)==32){
            save_img_=true;
        }

        if(save_img_){
            //file name
            std::stringstream stream;  
            if(saved_folder_==""){
                stream << "image"<< filenum_<< ".jpg";  
            }else{
                stream <<saved_folder_<< "/image"<< filenum_<< ".jpg";  
            }
            filenum_++;
            std::string filename = stream.str(); 
            cv::imwrite(filename,cv_ptr->image);
            save_img_=false;
            std::cout<<"Saved "<<filenum_<<" image.\n";
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");

    std::string topic_name="/camera/color/image_raw";
    std::string saved_folder="";
    if(argc>1){
        topic_name=argv[1];
    }
    if(argc>2){
        saved_folder=argv[2];
    }
    ImageConverter ic(topic_name, saved_folder);
    
    ros::spin();
    return 0;
}