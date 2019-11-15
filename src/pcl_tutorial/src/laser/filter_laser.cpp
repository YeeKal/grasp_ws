#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/rate.h>
#include <sensor_msgs/LaserScan.h> 
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"
#include "obj_srv/obj_6d.h"

#define PI 3.1415926
#define COUNTER_WIDTH 0.31
#define COUNTER_RADIUS 0.4
typedef pcl::PointXYZ PointT;

class FilterLaser{
public:
    ros::NodeHandle nh_;
    ros::Subscriber sub_scan_;      // subscriber to get laser data
    ros::Publisher pub_scan_;       // publisher for the laser data after processed
    ros::Publisher pub_line_;       // publisher for the detected line
    ros::Publisher pub_marker_;     // publisher for the markers in represent of lines
    ros::ServiceServer service_;    // service to give the counter pose
    tf::TransformBroadcaster broadcaster_;  // to publish the static transform between counter with laser
    geometry_msgs::Pose pose_;              // the counter pose

    pcl::PointCloud<PointT>::Ptr cloud_;
    pcl::PointCloud<PointT>::Ptr source_;
    pcl::KdTreeFLANN<PointT> kdtree_;

    double filter_distance_;    // point with a smaller distance to the source than this will be viewed as the source points
    bool is_first_;             // is the first frame of the topic
    bool pub_markers_;          // whether to publish line markers
    bool foud_counter_pose_;    // whether the counter pose is derivated
    double angle_min_;          // min angle of laser
    double angle_max_;          // max angle of laser
    double line_min_;           // the detected line length should be during (line_min, line_max_)
    double line_max_;
    double angle_increment_;    // angle increment of laser
    double max_radius_;         // the valid  detection area surrounding the robot(laser)
    int range_size_;            // data size of laser in each frame

    std::string frame_id_;
    std::string scan_topic_;
    std::string source_file_;
    

    line_extraction::LineExtraction line_extraction_;
    std::vector<line_extraction::Line> lines_;
    std::vector<unsigned int> filtered_indices_;    // Indices after filtering
    std::vector<double> range_data_;                // laser data

    visualization_msgs::Marker marker_people_;

    bool init_source_;



    FilterLaser(std::string source_file,bool init_source,double filter_distance=0.04)
        :cloud_(new pcl::PointCloud<PointT>),
        source_(new pcl::PointCloud<PointT>),
        filter_distance_(filter_distance),
        source_file_(source_file),
        is_first_(true),
        foud_counter_pose_(false),
        init_source_(init_source),
        max_radius_(2.0),
        line_min_(0.65),
        line_max_(0.75)
    {
        std::cout<<"Load source file ...\n";
        if(pcl::io::loadPCDFile<PointT>(source_file_,*source_)==-1){
            std::cout<<"Could not read source file:"<<source_file_<<std::endl;
            exit(1);
        }
        kdtree_.setInputCloud(source_);


        loadParameters();
        sub_scan_=nh_.subscribe(scan_topic_,100,&FilterLaser::laserCB,this);
        pub_scan_=nh_.advertise<sensor_msgs::LaserScan>("scan2",100);
        pub_line_ = nh_.advertise<laser_line_extraction::LineSegmentList>("line_segments", 1);
        if (pub_markers_){
            pub_marker_ = nh_.advertise<visualization_msgs::Marker>("line_markers", 1);
        }
        service_=nh_.advertiseService("recognize_counter", &FilterLaser::recognizeCounter,this);
    }

    void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        if(is_first_){
            angle_min_=msg->angle_min;
            angle_max_=msg->angle_max;
            angle_increment_=msg->angle_increment;
            range_size_=msg->ranges.size();//(int)((angle_max_-angle_min_)/angle_increment_);
            if(range_size_<10){
                std::cout<<"Invalid laser data.\n";
                exit(1);
            }
            is_first_=false;
            if(init_source_){
                is_first_=true;
                laser2PCloud(msg);
                pcl::io::savePCDFile(source_file_, *cloud_);
                ros::shutdown();
                exit(0);
            }
            cacheData(msg);
        }

        range_data_.assign(msg->ranges.begin(), msg->ranges.end());
        line_extraction_.setRangeData(range_data_);
    }
    
    void runOnce(){
        /* filter by source point cloud */
        PointT sp;
        std::vector<int> vec_ids(1);
        std::vector<float> vec_dis(1);
        double angle=angle_min_;
        std::vector<unsigned int> output;
        for(int i=0;i<range_size_;i++){
            angle +=angle_increment_*i;
            sp.x=range_data_[i]*cos(angle);
            sp.y=range_data_[i]*sin(angle);
            sp.z=0;
            if ( kdtree_.nearestKSearch (sp, 1, vec_ids, vec_dis) > 0 ){
                if(vec_dis[0]>filter_distance_){
                    output.push_back(i);
                }
            }
        }
        filtered_indices_=output;


        /* find lines */
        // change indices in line extraction
        line_extraction_.c_data_.indices=filtered_indices_;
        // filter by line_extraction and find lines
        line_extraction_.extractLines(lines_);
        // get back the indices filter by line_extraction
        filtered_indices_=line_extraction_.filtered_indices_; 


        detectCounterPose();

        detectSomethingElse();

        notifyAll(); //TODO: new thread
    }

    void detectSomethingElse(){
        std::vector<unsigned int> output;
        double x,y,cx,cy;
        cx=0;cy=0;
        double angle=angle_min_;
        int outliers=0;
        for(int i=0;i<filtered_indices_.size();i++){
            angle +=angle_increment_*filtered_indices_[i];
            x=range_data_[filtered_indices_[i]]*cos(angle);
            y=range_data_[filtered_indices_[i]]*sin(angle);
            // less than max_radius_ and not in the counter
            if(calLineLength(x,y,pose_.position.x,pose_.position.y)>COUNTER_RADIUS && calLineLength(x,y,0,0) <max_radius_ ){
                output.push_back(filtered_indices_[i]);
                outliers++;
                cx=cx*(outliers-1)/(double)outliers+x/(double)outliers;
                cy=cy*(outliers-1)/(double)outliers+y/(double)outliers;
            }
        }

        markerMsgElse(cx,cy,0.2,marker_people_);
        //debug
        std::cout<<"the remain points size:"<<output.size()<<std::endl;

    }
    void markerMsgElse(double cx,double cy,double radius, visualization_msgs::Marker& marker_msg){
        marker_msg.ns = "people";
        marker_msg.id = 1;
        marker_msg.type = visualization_msgs::Marker::CUBE;
        marker_msg.scale.x = 0.01;
        marker_msg.scale.y = 0.01;
        marker_msg.scale.z = 0.01;

        marker_msg.color.r = 1.0;
        marker_msg.color.g = 0.8;
        marker_msg.color.b = 0.2;
        marker_msg.color.a = 0.2;
        int num=20;
         marker_msg.points.clear();
        for (int i=0;i<num;i++)
        {
            geometry_msgs::Point p;
            p.x = cx+radius*cos(i*2*PI/num);
            p.y = cy+radius*sin(i*2*PI/num);
            p.z = 0;
            marker_msg.points.push_back(p);
        }
        marker_msg.header.frame_id = frame_id_;
        marker_msg.header.stamp = ros::Time::now();
    }

    void notifyAll(){

        // publish line
        laser_line_extraction::LineSegmentList line_msg;
        populateLineSegListMsg(lines_, line_msg);
        pub_line_.publish(line_msg);

        // Also publish markers if parameter publish_markers is set to true
        if (pub_markers_)
        {
            visualization_msgs::Marker marker_msg;
            populateMarkerMsg(lines_, marker_msg);
            pub_marker_.publish(marker_msg);
            pub_marker_.publish(marker_people_);
        }
    }

    void detectCounterPose(){
        double line_length=0;
        bool found=false;
        double angle=0;
        double radius;
        for (std::vector<line_extraction::Line>::const_iterator cit = lines_.begin(); cit != lines_.end(); ++cit){
            angle=cit->getAngle();
            radius=cit->getRadius();

            if(angle>PI/2 || angle<-PI/2) continue;
            if(radius>max_radius_) continue;
            line_length=calLineLength(
                             cit->getStart()[0],
                             cit->getStart()[1],
                             cit->getEnd()[0],
                             cit->getEnd()[1]
                            ); 
            if(line_length>line_max_ || line_length<line_min_) continue;
            // not greedy mode, one is enough
            // for debug
            //std::cout<<"line length:"<<line_length<<std::endl;
            found =true;

            Eigen::Quaterniond q;
            q=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ());
            pose_.position.x=(cit->getStart()[0]+cit->getEnd()[0]+COUNTER_WIDTH*cos(angle))/2;
            pose_.position.y=(cit->getStart()[1]+cit->getEnd()[1]+COUNTER_WIDTH*sin(angle))/2;
            pose_.position.z=0;
            pose_.orientation.x=q.x();
            pose_.orientation.y=q.y();
            pose_.orientation.z=q.z();
            pose_.orientation.w=q.w();

            //TODO:: IF NOT FOUND THE COUNTER POSE

            broadcaster_.sendTransform(
                tf::StampedTransform(
                tf::Transform(tf::Quaternion(q.x(),q.y(),q.z(),q.w()),tf::Vector3(pose_.position.x,pose_.position.y,pose_.position.z)),
                ros::Time::now(),"laser","counter"));
            
            break;
        }
        foud_counter_pose_=found;

    }
    bool recognizeCounter(obj_srv::obj_6d::Request &req,obj_srv::obj_6d::Response &res){
        

        // construct req
        if(req.start){
            geometry_msgs::PoseArray obj_array;
            obj_array.poses.resize(0);
            if(foud_counter_pose_){
                obj_array.poses.push_back(pose_);
            }
            res.obj_array = obj_array;

        }
        return true;
    }

    static double calLineLength(float x1,float y1,float x2,float y2){
        return (double)sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    }

    /*
    [x,y,z]=[ r * cos theta,  r * sin theta , 0]
    */
    void laser2PCloud(const sensor_msgs::LaserScan::ConstPtr& msg){
        double angle=angle_min_;
        for(int i=0;i<range_size_;i++){
            angle +=angle_increment_*i;
            cloud_->points[i].x = msg->ranges[i]*cos(angle);
            cloud_->points[i].y = msg->ranges[i]*sin(angle);
            cloud_->points[i].z = 0;
        }
    }

    // Members
    void loadParameters();
    void populateLineSegListMsg(const std::vector<line_extraction::Line>&, laser_line_extraction::LineSegmentList&);
    void populateMarkerMsg(const std::vector<line_extraction::Line>&, visualization_msgs::Marker&);
    void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
};



int main(int argc,char ** argv){
    ros::init(argc, argv, "read_laser");
    ros::NodeHandle nh;
    double angle_min=-3.14;
    double angle_max=3.14;
    std::string source_file;
    bool init_source;
    nh.param<std::string>("source_file", source_file, "source.pcd");
    nh.param<bool>("init_source", init_source, false);

    ros::Rate rate(20);

    FilterLaser FilterLaser(source_file,init_source);

    if(init_source){
        rate.sleep();
        ros::spinOnce();
        return 0;
    }

    while (ros::ok())
    {
        FilterLaser.runOnce();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

void FilterLaser::loadParameters(){
    ROS_DEBUG("*************************************");
    ROS_DEBUG("PARAMETERS:");

    // Parameters used by this node
    
    std::string frame_id, scan_topic;
    bool pub_markers;

    nh_.param<std::string>("frame_id", frame_id, "laser");
    frame_id_ = frame_id;
    ROS_DEBUG("frame_id: %s", frame_id_.c_str());

    nh_.param<std::string>("scan_topic", scan_topic, "scan");
    scan_topic_ = scan_topic;
    ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

    nh_.param<bool>("publish_markers", pub_markers, false);
    pub_markers_ = pub_markers;
    ROS_DEBUG("publish_markers: %s", pub_markers ? "true" : "false");

    // Parameters used by the line extraction algorithm

    double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
            max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
    int min_line_points;

    nh_.param<double>("bearing_std_dev", bearing_std_dev, 1e-3);
    line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
    ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

    nh_.param<double>("range_std_dev", range_std_dev, 0.02);
    line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
    ROS_DEBUG("range_std_dev: %f", range_std_dev);

    nh_.param<double>("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
    line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
    ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);
    
    nh_.param<double>("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
    line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
    ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

    nh_.param<double>("max_line_gap", max_line_gap, 0.4);
    line_extraction_.setMaxLineGap(max_line_gap);
    ROS_DEBUG("max_line_gap: %f", max_line_gap);

    nh_.param<double>("min_line_length", min_line_length, 0.5);
    line_extraction_.setMinLineLength(min_line_length);
    ROS_DEBUG("min_line_length: %f", min_line_length);

    nh_.param<double>("min_range", min_range, 0.4);
    line_extraction_.setMinRange(min_range);
    ROS_DEBUG("min_range: %f", min_range);

    nh_.param<double>("min_split_dist", min_split_dist, 0.05);
    line_extraction_.setMinSplitDist(min_split_dist);
    ROS_DEBUG("min_split_dist: %f", min_split_dist);

    nh_.param<double>("outlier_dist", outlier_dist, 0.05);
    line_extraction_.setOutlierDist(outlier_dist);
    ROS_DEBUG("outlier_dist: %f", outlier_dist);

    nh_.param<int>("min_line_points", min_line_points, 9);
    line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
    ROS_DEBUG("min_line_points: %d", min_line_points);

    ROS_DEBUG("*************************************");
}

void FilterLaser::populateLineSegListMsg(const std::vector<line_extraction::Line>& lines, laser_line_extraction::LineSegmentList& line_msgs){
    for (std::vector<line_extraction::Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        laser_line_extraction::LineSegment line_msg;
        line_msg.angle = cit->getAngle(); 
        line_msg.radius = cit->getRadius(); 
        line_msg.covariance = cit->getCovariance(); 
        line_msg.start = cit->getStart(); 
        line_msg.end = cit->getEnd(); 
        line_msgs.line_segments.push_back(line_msg);
    }
    line_msgs.header.frame_id = frame_id_;
    line_msgs.header.stamp = ros::Time::now();
}
void FilterLaser::populateMarkerMsg(const std::vector<line_extraction::Line>& lines, visualization_msgs::Marker& marker_msg){
    marker_msg.ns = "line_extraction";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    marker_msg.scale.x = 0.1;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    for (std::vector<line_extraction::Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        geometry_msgs::Point p_start;
        p_start.x = cit->getStart()[0];
        p_start.y = cit->getStart()[1];
        p_start.z = 0;
        marker_msg.points.push_back(p_start);
        geometry_msgs::Point p_end;
        p_end.x = cit->getEnd()[0];
        p_end.y = cit->getEnd()[1];
        p_end.z = 0;
        marker_msg.points.push_back(p_end);
    }
    marker_msg.header.frame_id = frame_id_;
    marker_msg.header.stamp = ros::Time::now();
}

void FilterLaser::cacheData(const sensor_msgs::LaserScan::ConstPtr& msg){
    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;
    const std::size_t num_measurements = std::ceil(
        (msg->angle_max - msg->angle_min) / msg->angle_increment);
    for (std::size_t i = 0; i < num_measurements; ++i)
    {
    const double b = msg->angle_min + i * msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
    }

    line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
    ROS_DEBUG("Data has been cached.");
}