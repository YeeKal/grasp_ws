//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <obj6d_msgs/rgbd.h>
#include <obj_srv/obj_6d.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "ppf.hpp"
#include "object6D.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#define voting_thred 300

using namespace std;
using namespace cv;
using namespace ppf;
using namespace obj6D;


const double camera_cx = 310.95;//325.5//319.5 310.95 310.95
const double camera_cy = 234.74;//253.5//239.5 234.74 234.74
const double camera_fx = 615.377;//518.0//570.3422(openni2) 615.377
const double camera_fy = 615.377;//519.0//570.3422(openni2) 615.377
ppf::PPF6DDetector detector(0.06, 9.0, 9.0, 0.03); //0.05, 9, 12, 0.03
pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb (new pcl::PointCloud<pcl::PointXYZRGBA> ());
pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
vector<Pose6DPtr> results;

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
            geometry_msgs::Pose pose;
            pose.position.x = (float)t[0];
            pose.position.y = (float)t[1];
            pose.position.z = (float)t[2];
            pose.orientation.x = (float)q[0];
            pose.orientation.y = (float)q[1];
            pose.orientation.z = (float)q[2];
            pose.orientation.w = (float)q[3];
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

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        help("Not enough input arguments");
        exit(1);
    }
    ros::init(argc, argv, "ppf_main");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<obj6d_msgs::rgbd>("data_service");
    ros::ServiceServer service = nh.advertiseService("obj_start", obj_s);
    obj6d_msgs::rgbd srv;
    srv.request.start = true;
    sensor_msgs::Image msg_rgb;
    sensor_msgs::Image msg_depth;

    string modelFileName = (string)argv[1];
    string path = ros::package::getPath("ppf");
    stringstream ss;
    ss<<path<<"/model/"<<modelFileName;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr model_with_normals (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::io::loadPLYFile(ss.str().c_str(),*model);
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
            rgb = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3)->image;
            depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1)->image;

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
                if(!(depth.at<short>(r,c)>0)){continue;}
                double scene_z = double(depth.at<short>(r,c))/1000.0;
                double scene_x = (c - camera_cx) * scene_z / camera_fx;
                double scene_y = (r - camera_cy) * scene_z / camera_fy;
                p.x = scene_x;
                p.y = scene_y;
                p.z = scene_z;
                p_rgb.x = scene_x;
                p_rgb.y = scene_y;
                p_rgb.z = scene_z;
                p_rgb.b = rgb.ptr<uchar>(r)[c*3];
                p_rgb.g = rgb.ptr<uchar>(r)[c*3+1];
                p_rgb.r = rgb.ptr<uchar>(r)[c*3+2];
                scene->points.push_back(p);
                scene_rgb->points.push_back(p_rgb);
            }
        }
        detector.preProcessing(scene, *scene_with_normals, 0);

        /*viewer1->removeAllPointClouds();
        viewer1->addPointCloudNormals<pcl::PointNormal> (model_with_normals, 10, 0.01, "normals");
        viewer1->addPointCloud<pcl::PointNormal> (model_with_normals, single_color1, "sample cloud1");
        viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");*/
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
