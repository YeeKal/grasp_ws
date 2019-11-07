//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <obj_srv/rgbd_image.h>
#include <obj_srv/obj_6d.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
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

#include "pcl_tutorial/hsv_segmentation.h"
#include "super4pcs_icp.h"
#include <obj_srv/rgbd_image.h>
#include <obj_srv/obj_6d.h>



#define voting_thred 80

using namespace std;
using namespace cv;


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const double camera_cx = 319.5;//325.5//319.5 310.95 310.95
const double camera_cy = 239.5;//253.5//239.5 234.74 234.74
const double camera_fx = 570.3422;//518.0//570.3422(openni2) 615.377
const double camera_fy = 570.3422;//519.0//570.3422(openni2) 615.377
pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
vector<Eigen::Matrix4f> results;
pcl::Super4pcsICP<PointT,PointT> sup_icp;


// service call back function
bool recognizeBlock(obj_srv::obj_6d::Request &req,obj_srv::obj_6d::Response &res)
{
    if(req.start)
    {
        cout << "Starting matching..." << endl;
        results.clear();

        uint tick1 = cv::getTickCount();
        sup_icp.setInputSource(scene);
    	sup_icp.matchDebug();
        uint tick2 = cv::getTickCount();

        cout << "Matching complete in "
             << (double)(tick2-tick1)/ cv::getTickFrequency()
             << " sec" << endl;
        results.push_back(sup_icp.transformation_);
        cout<<"results.size: "<<results.size()<<endl;
        cout<<"acores: "<<sup_icp.score_<<endl;

        geometry_msgs::PoseArray obj_array;
        obj_array.poses.resize(0);
        for(int i=0;i<results.size();i++)
        {
            Eigen::Matrix3f rot=results[i].block<3,3>(0,0);
            Eigen::Quaternionf q=Eigen::Quaternionf(rot);

            geometry_msgs::Pose pose;
            pose.position.x = (float)results[i](0,3);
            pose.position.y = (float)results[i](1,3);
            pose.position.z = (float)results[i](2,3);
            pose.orientation.x = (float)q.x();
            pose.orientation.y = (float)q.y();
            pose.orientation.z = (float)q.z();
            pose.orientation.w = (float)q.w();
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
bool loadPointCloud(std::string argv,PointCloudT::Ptr cloud){
	if(argv.substr(argv.length()-3,argv.length())=="pcd"){
		if(pcl::io::loadPCDFile<PointT> (argv, *cloud)>=0){
			return true;
		}
	}
	else if(argv.substr(argv.length()-3,argv.length())=="ply"){
		if(pcl::io::loadOBJFile<PointT> (argv, *cloud)>=0){
			return true;
		}
	}
	else if(argv.substr(argv.length()-3,argv.length())=="obj"){
		if(pcl::io::loadOBJFile<PointT> (argv, *cloud)>=0)
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

// rosrun pcl_tutorial main  block/edit_block/del2_t.pcd  -o 0.7 -d 0.01 -t 1000 -n 200 -it 20


int main(int argc, char** argv)
{
	
    if (argc < 2)
    {
        help("Not enough input arguments");
        exit(1);
    }
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<obj_srv::rgbd_image>("get_image");
    ros::ServiceServer service = nh.advertiseService("recognize_block", recognizeBlock);
    obj_srv::rgbd_image srv;
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;

    //HSVSegmentation hsv(85,125,20,255,20,255);
    HSVSegmentation hsv(100,123, 150,255,85,220);
    sup_icp.initMatcher(argc,argv);

	
    PointCloudT::Ptr model (new PointCloudT());
    if(!loadPointCloud(argv[1],model)){
        return -1;
    }
    sup_icp.setInputTarget(model);


    //pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer());
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("Super4PCS+ICP demo"));
    viewer->addCoordinateSystem(1);
    int v1 (0);
	int v2 (1);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    results.clear();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (scene, (int) 255 , (int) 255,
																				(int) 255);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color1(model_with_normals, 0, 255, 0);
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color2(scene_with_normals, 0, 255, 0);

    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        Mat rgb,depth,depth_copy;
        client.call(srv);
        try
        {
            msg_rgb = srv.response.rgb_image;
            msg_depth = srv.response.depth_image;
            rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8)->image;
            depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return 1;
        }

        scene->points.clear();
        scene_rgb->points.clear();

        depth_copy=depth.clone();
        hsv.setSrc(rgb);
        hsv.getDepthDst(depth);
        //scene
        for (int r=0;r<rgb.rows;r++)
        {
            for (int c=0;c<rgb.cols;c++)
            {
                pcl::PointXYZ p;
                if(!(depth.at<float>(r,c)>0&&depth.at<float>(r,c)<1.3)){continue;}
                double scene_z = double(depth.at<float>(r,c));
                double scene_x = (c - camera_cx) * scene_z / camera_fx;
                double scene_y = (r - camera_cy) * scene_z / camera_fy;
                p.x = scene_x;
                p.y = scene_y;
                p.z = scene_z;
                scene->points.push_back(p);
            }
        }
        //scene_rgb
        for (int r=0;r<rgb.rows;r++)
        {
            for (int c=0;c<rgb.cols;c++)
            {
                pcl::PointXYZRGBA p_rgb;
                //scene
                if(!(depth_copy.at<float>(r,c)>0&&depth_copy.at<float>(r,c)<1.3)){continue;}
                double scene_z = double(depth_copy.at<float>(r,c));
                double scene_x = (c - camera_cx) * scene_z / camera_fx;
                double scene_y = (r - camera_cy) * scene_z / camera_fy;
                p_rgb.x = scene_x;
                p_rgb.y = scene_y;
                p_rgb.z = scene_z;
                p_rgb.r = rgb.ptr<uchar>(r)[c*3+2];
                p_rgb.g = rgb.ptr<uchar>(r)[c*3+1];
                p_rgb.b = rgb.ptr<uchar>(r)[c*3];
                scene_rgb->points.push_back(p_rgb);
            }
        }

        viewer->removeAllPointClouds();
	    viewer->addPointCloud (scene, cloud_in_color_h, "scene", v1);
        viewer->addPointCloud(scene_rgb, "scene_rgb",v2);
        if(results.size()>0)
        {
            //viewer->addPointCloud<pcl::PointNormal> (scene_with_normals, single_color2, "sample cloud1");
            //viewer->addPointCloudNormals<pcl::PointNormal> (scene_with_normals, 10, 0.01, "normals");
            Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity ();
            PointCloudT::Ptr cloud_ppf (new PointCloudT);
            for (size_t i=0; i<results.size(); i++)
            {
                transformation = results[i].inverse();
                //cout<<transformation<<endl;
                pcl::transformPointCloud(*model, *cloud_ppf, transformation);
                pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_ppf, 0, 255, 0);
                stringstream ss;
                ss<<i;
                //viewer->addPointCloudNormals<pcl::PointNormal> (cloud_ppf1, 10, 0.01, "normals2");
                viewer->addPointCloud<PointT> (cloud_ppf, single_color, ss.str().c_str(),v2);
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ss.str().c_str());
            }
        }
        //viewer1->spinOnce();
        viewer->spinOnce();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
