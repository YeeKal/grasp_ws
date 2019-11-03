//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <obj_srv/rgbd_image.h>
#include <obj_srv/obj_6d.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ppf.hpp"
#include "object6D.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>

#define voting_thred 80

using namespace std;
using namespace cv;
using namespace ppf;
using namespace obj6D;


const double camera_cx = 319.5;//325.5//319.5 310.95 310.95
const double camera_cy = 239.5;//253.5//239.5 234.74 234.74
const double camera_fx = 570.3422;//518.0//570.3422(openni2) 615.377
const double camera_fy = 570.3422;//519.0//570.3422(openni2) 615.377
ppf::PPF6DDetector detector(0.06, 9.0, 9.0, 0.03); //0.05, 9, 12, 0.03
pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
vector<Pose6DPtr> results;

class HSVSegmentation{
public:
	cv::Mat src_,bgr_,hsv_;
	cv::Mat mask_,final_mask_;
	int hmin_,hmax_,smin_,smax_,vmin_,vmax_;
	HSVSegmentation(int hmin,int hmax,int smin,int smax,int vmin,int vmax)
	:hmin_(hmin),hmax_(hmax),smin_(smin),smax_(smax),vmin_(vmin),vmax_(vmax)
	{}
	~HSVSegmentation(){}
	//Mat对象做为函数参数，不论是传引用，传指针，对于传值调用，在函数内对其进行的操作会作用到原对象
	void setSrc(cv::Mat src){
		src_=src;//not need deep copy
		mask_ = Mat::zeros(src_.rows, src_.cols, CV_8UC1);
		final_mask_ = Mat::zeros(src_.rows, src_.cols, CV_8UC1);
		segProcess();
	}
	void segProcess(){
		cv::GaussianBlur(src_, bgr_, cv::Size(5, 5), 3, 3);
		cv::cvtColor(bgr_, hsv_, CV_BGR2HSV);
		cv::inRange(hsv_, Scalar(hmin_, smin_, vmin_), Scalar(hmax_, smax_, vmax_), mask_);

		//open close
		cv::Mat element=getStructuringElement(MORPH_RECT, Size(15,15));
		cv::morphologyEx(mask_, mask_, MORPH_OPEN, element);

		//contour
		vector<vector<cv::Point> > contours;
		cv::findContours(mask_,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); 
		// 寻找最大连通域  
		double maxArea = 0;  
		int max_contour=0;
		vector<cv::Point> maxContour;  
		for(size_t i = 0; i < contours.size(); i++)  
		{  
			double area = cv::contourArea(contours[i]);  
			if (area > maxArea)  
			{  
				maxArea = area;  
				max_contour=i; 
			}  
		}  

		maxContour=contours[max_contour];
		drawContours(final_mask_, contours, max_contour, Scalar(255), CV_FILLED);
	}

	void getMask(Mat &mask){
		mask=final_mask_.clone();
	}

	void getColorDst(Mat &img){
		Mat dst;
		dst = Mat::zeros(src_.size(), src_.type());
		for (int r = 0; r < src_.rows; r++)
		{
			for (int c = 0; c < src_.cols; c++)
			{
				if (final_mask_.at<uchar>(r, c) == 255)
				{
					dst.at<Vec3b>(r, c) = src_.at<Vec3b>(r, c);
				}
			}
		}
		img= dst.clone();
	}

	// filter the depth img by the mask
	void getDepthDst(Mat &img){
		Mat dst;
		dst = Mat::zeros(img.size(),img.type());
		for (int r = 0; r < img.rows; r++)
		{
			for (int c = 0; c < img.cols; c++)
			{
				if (final_mask_.at<uchar>(r, c) == 255)
				{
					dst.at<float>(r, c) = img.at<float>(r, c);
				}
			}
		}
		img= dst.clone();
	}

};


bool obj_s(obj_srv::obj_6d::Request &req,obj_srv::obj_6d::Response &res)
{
    if(req.start)
    {
        cout << "Starting matching..." << endl;
        results.clear();
        uint tick1 = cv::getTickCount();
        detector.match(scene_with_normals, results, 1.0f/5.0f);
        uint tick2 = cv::getTickCount();
        cout << "Matching complete in "
             << (double)(tick2-tick1)/ cv::getTickFrequency()
             << " sec" << endl;
        geometry_msgs::PoseArray obj_array;
        obj_array.poses.resize(0);
        cout<<"results.size: "<<results.size()<<endl;
        cout<<"Max numVotes: "<<results[0]->numVotes<<endl;
        for(int i=0;i<results.size();i++)
        {
            if(results[i]->numVotes<voting_thred){continue;}
            Vec3d t = results[i]->pose_t;
            Vec4d q = results[i]->q;
			std::cout<<results[i]->pose_R<<std::endl;
            geometry_msgs::Pose pose;
            pose.position.x = (float)t[0];
            pose.position.y = (float)t[1];
            pose.position.z = (float)t[2];
            pose.orientation.x = (float)q[1];
            pose.orientation.y = (float)q[2];
            pose.orientation.z = (float)q[3];
            pose.orientation.w = (float)q[0];
            obj_array.poses.push_back(pose);
        }
        res.obj_array = obj_array;
    }
    return true;
}
static void help(const string& errorMessage)
{
    cout << "Program init error : "<< errorMessage << endl;
    cout << "\nUsage : rosrun ppf ppf_main [input model file]"<< endl;
    cout << "\nPlease start again with new parameters"<< endl;
}
void getModelName(int argc, char** argv,pcl::PointCloud<pcl::PointXYZ>::Ptr model){
    string modelFileName = (string)argv[1];
    string path = ros::package::getPath("ppf");
    stringstream ss;
    ss<<path<<"/model/"<<modelFileName;
    
    std::vector<int> ply_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
    std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");


    if(ply_file_indices.size()>0){
        pcl::PolygonMesh mesh;
        pcl::io::loadPLYFile(ss.str().c_str(), mesh);
        pcl::fromPCLPointCloud2(mesh.cloud, *model);
    }else if(pcd_file_indices.size()>0){
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss.str().c_str(),*model)==-1){
            PCL_ERROR("could not read file\n");
            //return (-1);
        }
	}else{
        std::cout<<"Invalid file types.\n";
        //return 0;
    }
}

void filterScene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene){
    

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    seg.setInputCloud (scene);
    seg.segment (*inliers, *coefficients);
    //std::cout<<"inlier size:"<<inliers->indices.size ()<<std::endl;
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        //return 0;
    }

    // Extract the inliers
    extract.setInputCloud (scene);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*scene);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (scene);
    // pass.setFilterFieldName ("b");
    // pass.setFilterLimits (10, 255);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.2, 1.5);
    pass.filter (*scene);
    // pass.setFilterFieldName ("x");
    // pass.setInputCloud (scene);
    // pass.setFilterLimits (-0.2, 0.4);
    // pass.filter (*scene);
}

int main(int argc, char** argv)
{
	
    if (argc < 2)
    {
        help("Not enough input arguments");
        exit(1);
    }
    ros::init(argc, argv, "ppf_main");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<obj_srv::rgbd_image>("get_image");
    ros::ServiceServer service = nh.advertiseService("obj_start", obj_s);
    obj_srv::rgbd_image srv;
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;

    HSVSegmentation hsv(85,125,20,255,20,255);


    string modelFileName = (string)argv[1];
    string path = ros::package::getPath("ppf");
    stringstream ss;
    ss<<path<<"/model/"<<modelFileName;
	
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
    //pcl::io::loadPLYFile(ss.str().c_str(),*model);
    getModelName(argc,argv,model);
    cout << "Pre-Processing..." << endl;
    int64 tick1 = cv::getTickCount();
    detector.preProcessing(model, *model_with_normals, 1);
    int64 tick2 = cv::getTickCount();
    cout << "Pre-Processing complete in "
         << (double)(tick2-tick1)/ cv::getTickFrequency()
         << " sec" << endl;
    cout << "Training..." << endl;
    tick1 = cv::getTickCount();
    detector.trainModel(model_with_normals);
    tick2 = cv::getTickCount();
    cout << "Training complete in "
         << (double)(tick2-tick1)/ cv::getTickFrequency()
         << " sec" << endl;
    //pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer());
    pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer());
    //viewer1->addCoordinateSystem(0.1);
    viewer2->addCoordinateSystem(0.1);
    results.clear();
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color1(model_with_normals, 0, 255, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color2(scene_with_normals, 0, 255, 0);

    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        Mat rgb,depth;
        client.call(srv);
        try
        {
            msg_rgb = srv.response.rgb_image;
            msg_depth = srv.response.depth_image;
            rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8)->image;
            depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            hsv.setSrc(rgb);
            //hsv.getMask(final_mask);
            hsv.getDepthDst(depth);
            // imshow("rgb",rgb);
            // imshow("depth",depth);
            waitKey(0);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return 1;
        }
        scene->points.clear();
        scene_rgb->points.clear();
        scene_with_normals->points.clear();
        for (int r=0;r<rgb.rows;r++)
        {
            for (int c=0;c<rgb.cols;c++)
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
                p_rgb.r = rgb.ptr<uchar>(r)[c*3+2];
                p_rgb.g = rgb.ptr<uchar>(r)[c*3+1];
                p_rgb.b = rgb.ptr<uchar>(r)[c*3];
                scene->points.push_back(p);
                scene_rgb->points.push_back(p_rgb);
            }
        }
        //filterScene(scene);
        //filterScene(scene_rgb);
        detector.preProcessing(scene, *scene_with_normals, 0);


        viewer2->removeAllPointClouds();
        //viewer2->addPointCloud<pcl::PointNormal> (scene_with_normals, single_color2, "sample cloud1");
        //viewer2->addPointCloudNormals<pcl::PointNormal> (scene_with_normals, 10, 0.01, "normals");
        viewer2->addPointCloud(scene_rgb, "scene_rgb");
        if(results.size()>0)
        {
            //viewer2->addPointCloud<pcl::PointNormal> (scene_with_normals, single_color2, "sample cloud1");
            //viewer2->addPointCloudNormals<pcl::PointNormal> (scene_with_normals, 10, 0.01, "normals");
            Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity ();
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ppf (new pcl::PointCloud<pcl::PointNormal>);
            for (size_t i=0; i<results.size(); i++)
            {
                Pose6DPtr result = results[i];
                if(result->numVotes<voting_thred){continue;}
                //cout<<"numVotes: "<<result->numVotes<<endl;
                //cout << "Pose Result " << i << endl;
                transformation = cvMat2Eigen(result->pose);
                //cout<<transformation<<endl;
                pcl::transformPointCloud(*model_with_normals, *cloud_ppf, transformation);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(cloud_ppf, 0, 255, 0);
                stringstream ss;
                ss<<i;
                //viewer2->addPointCloudNormals<pcl::PointNormal> (cloud_ppf1, 10, 0.01, "normals2");
                viewer2->addPointCloud<pcl::PointNormal> (cloud_ppf, single_color, ss.str().c_str());
                viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str().c_str());
            }
        }
        //viewer1->spinOnce();
        viewer2->spinOnce();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
