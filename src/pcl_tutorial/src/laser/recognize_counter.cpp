#include <iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <boost/array.hpp>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/rate.h>
#include <sensor_msgs/LaserScan.h> 
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "laserline/line_feature.h"
#include "obj_srv/obj_6d.h"

#define COUNTER_WIDTH 0.31
#define COUNTER_RADIUS 0.4
#define NZEROS 4
#define NPOLES 4
#define GAIN   2.072820954e+02
typedef pcl::PointXYZ PointT;

class FilterLaser{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::Subscriber sub_scan_;      // subscriber to get laser data
    ros::Publisher pub_scan_;       // publisher for the laser data after processed
    ros::Publisher pub_line_;       // publisher for the detected line
    ros::Publisher pub_pose2d_;       // publisher for the detected line
    ros::Publisher pub_line_marker_;     // publisher for the markers in represent of lines
    ros::Publisher pub_people_marker_;     // publisher for the markers in represent of lines
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
    bool found_people_;
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
    
    line_feature::LineFeature line_feature_;
    //line_extraction::LineExtraction line_extraction_;
    std::vector<line> lines_;
	std::vector<gline> glines_;
    std::vector<unsigned int> filtered_indices_;    // Indices after filtering
    std::vector<double> range_data_;                // laser data
    std::vector<double> range_cur_;                // laser data


    visualization_msgs::Marker marker_people_;
    sensor_msgs::LaserScan data_cur_;
    geometry_msgs::Pose2D pose2d_;

    bool init_source_;



    FilterLaser(std::string source_file,bool init_source,double filter_distance=0.04,ros::NodeHandle nh=ros::NodeHandle(), ros::NodeHandle nh_local=ros::NodeHandle())
        :nh_(nh),
	    nh_local_(nh_local),
        cloud_(new pcl::PointCloud<PointT>),
        source_(new pcl::PointCloud<PointT>),
        filter_distance_(filter_distance),
        source_file_(source_file),
        is_first_(true),
        foud_counter_pose_(false),
        found_people_(false),
        init_source_(init_source),
        max_radius_(2.0),
        line_min_(0.65),
        line_max_(0.75)
    {
        
        if(!init_source_){
            std::cout<<"Load source file ...\n";
            if(pcl::io::loadPCDFile<PointT>(source_file_,*source_)==-1){
                std::cout<<"Could not read source file:"<<source_file_<<std::endl;
                exit(1);
            }
            kdtree_.setInputCloud(source_);
        }


        loadParameters();
        sub_scan_=nh_.subscribe(scan_topic_,100,&FilterLaser::laserCB,this);
        pub_scan_=nh_.advertise<sensor_msgs::LaserScan>("scan2",100);
        pub_pose2d_=nh_.advertise<geometry_msgs::Pose2D>("counter_pose2d",50);
        if (pub_markers_){
            pub_line_marker_ = nh_.advertise<visualization_msgs::Marker>("line_markers", 10);
            pub_people_marker_=nh_.advertise<visualization_msgs::Marker>("people_markers", 10);
        }
        service_=nh_.advertiseService("recognize_counter", &FilterLaser::recognizeCounter,this);
    }

    void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        if(is_first_){
            std::cout<<"is first\n";
            angle_min_=msg->angle_min;
            angle_max_=msg->angle_max;
            angle_increment_=msg->angle_increment;
            range_size_=msg->ranges.size();//(int)((angle_max_-angle_min_)/angle_increment_);
            if(range_size_<10){
                std::cout<<"Invalid laser data.\n";
                exit(1);
            }
            if(init_source_){
                laser2PCloud(msg);
                pcl::io::savePCDFile(source_file_, *cloud_);
                std::cout<<"Saved source file"<<std::endl;
                ros::shutdown();
                exit(0);
            }
            cacheData(msg);

            data_cur_.angle_min=angle_min_;
            data_cur_.angle_max=angle_max_;
            data_cur_.angle_increment=msg->angle_increment;
            data_cur_.time_increment=msg->time_increment;
            data_cur_.header.frame_id="laser";
            data_cur_.scan_time=msg->scan_time;
            data_cur_.range_min=msg->range_min;
            data_cur_.range_max=msg->range_max;

            is_first_=false;
        }

        range_data_.assign(msg->ranges.begin(), msg->ranges.end());
        

        //data_cur_=*msg;
        data_cur_.ranges=msg->ranges;
        data_cur_.intensities=msg->intensities;
        data_cur_.header.stamp=ros::Time();
    }
    
    void runOnce(){
        /* filter by source point cloud */
        if(is_first_) return;
        range_cur_.assign(range_data_.begin(), range_data_.end());
        PointT sp;
        std::vector<int> vec_ids(1);
        std::vector<float> vec_dis(1);
        double angle=angle_min_;
        std::vector<unsigned int> output;
        for(int i=0;i<range_size_;i++){
            angle +=angle_increment_;

            // point distance less than max_radius+1.0
            if(range_cur_[i]>max_radius_ || range_cur_[i]<0.1){
                data_cur_.ranges[i]=0;
                data_cur_.intensities[i]=0;
                continue;
            }
            sp.x=range_cur_[i]*cos(angle);
            sp.y=range_cur_[i]*sin(angle);
            sp.z=0;
            if ( kdtree_.nearestKSearch (sp, 1, vec_ids, vec_dis) > 0 ){
                if(vec_dis[0]>filter_distance_-0.01){
                    output.push_back(i);
                    continue;
                }
                data_cur_.ranges[i]=0;
                data_cur_.intensities[i]=0;
            }
            
        }
        filtered_indices_=output;
        //printVector(filtered_indices_,"1st filter");
        line_feature_.setRangeData(range_cur_);
        //printVector(filtered_indices_,"1st filter");
        /* find lines */
        line_feature_.extractLinesWithIndex(lines_,glines_,filtered_indices_);
        // get back the indices filter by line_extraction
        //filtered_indices_=line_extraction_.filtered_indices_; 


        detectCounterPose();
        detectSomethingElse();
        notifyAll(); //TODO: new thread
        //std::cout<<"hahad\n";

    }
    void printVector(std::vector<unsigned int> a, std::string name){
        std::cout<<name<<" with "<<a.size()<<" points."<<std::endl;
        std::cout<<"data:";
        for(int i=0;i<a.size();i++){
            if(i%30==0) std::cout<<std::endl;
            std::cout<<a[i]<<" ";
        }
        std::cout<<std::endl;
    }

    void detectSomethingElse(){
        std::vector<unsigned int> output;
        double x,y,cx,cy;
        cx=0;cy=0;
        double angle=angle_min_;
        int outliers=0;
        for(int i=0;i<filtered_indices_.size();i++){
            angle =angle_min_+angle_increment_*filtered_indices_[i];
            x=range_cur_[filtered_indices_[i]]*cos(angle);
            y=range_cur_[filtered_indices_[i]]*sin(angle);
            //std::cout<<i<<":"<<filtered_indices_[i]<<" "<<"x:"<<x<<"  y:"<<y<<std::endl;
            // less than max_radius_ and not in the counter
            if(foud_counter_pose_){
                if(calLineLength(x,y,pose_.position.x,pose_.position.y)>COUNTER_RADIUS && calLineLength(x,y,0,0) <max_radius_ ){
                    output.push_back(filtered_indices_[i]);
                    outliers++;
                    cx=cx*(outliers-1)/(double)outliers+x/(double)outliers;
                    cy=cy*(outliers-1)/(double)outliers+y/(double)outliers;
                }
                //only counter exists,  people matters
            // else{
            //     if(calLineLength(x,y,0,0) <max_radius_ +1.0){
            //         output.push_back(filtered_indices_[i]);
            //         outliers++;
            //         cx=cx*(outliers-1)/(double)outliers+x/(double)outliers;
            //         cy=cy*(outliers-1)/(double)outliers+y/(double)outliers;
            //     }
            }
        }
        if(output.size()>20){
            found_people_=true;
        }else{
            found_people_=false;
        }
        //std::cout<<"cx:"<<cx<<"  cy:"<<cy<<std::endl;
        markerMsgElse(cx,cy,0.4,marker_people_);
        //debug
        //std::cout<<"the remain points size:"<<output.size()<<std::endl;

    }
    void markerMsgElse(double cx,double cy,double radius, visualization_msgs::Marker& marker_msg){
        marker_msg.ns = "people";
        marker_msg.id = 1;
        marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
        marker_msg.scale.x = 0.05;
        marker_msg.scale.y = 0.05;
        marker_msg.scale.z = 0.05;

        marker_msg.color.r = 1.0;
        marker_msg.color.g = 0.8;
        marker_msg.color.b = 0.2;
        marker_msg.color.a = 1.0;
        int num=20;
        marker_msg.points.clear();
        if(found_people_){
            for (int i=0;i<num;i++)
            {
                geometry_msgs::Point p;
                p.x = cx+radius*cos(i*2*PI/num);
                p.y = cy+radius*sin(i*2*PI/num);
                p.z = 0;
                marker_msg.points.push_back(p);
            }
        }
        marker_msg.header.frame_id = frame_id_;
        marker_msg.header.stamp = ros::Time::now();
    }

    void notifyAll(){
        // publish line
        pub_scan_.publish(data_cur_);
        if(foud_counter_pose_) pub_pose2d_.publish(pose2d_);

        // Also publish markers if parameter publish_markers is set to true
        if (pub_markers_)
        {
            visualization_msgs::Marker marker_msg;
            populateMarkerMsg(glines_, marker_msg);
            pub_line_marker_.publish(marker_msg);
            pub_people_marker_.publish(marker_people_);
        }
    }

    void detectCounterPose(){
        double line_length=0;
        bool found=false;
        double angle=0;
        double radius;
        //std::cout<<"line size:"<<glines_.size()<<std::endl;
        for (std::vector<gline>::const_iterator cit = glines_.begin(); cit != glines_.end(); ++cit){
            std::vector<double> start(2),end(2);
            start[0]=cit->x1;
            start[1]=cit->y1;
            end[0]=cit->x2;
            end[1]=cit->y2;

            angle=calLineAngle(start[0],start[1],end[0],end[1]);
            radius=calLineRadius(start[0],start[1],end[0],end[1],angle);
            

            if(angle>PI/2 || angle<-PI/2) continue;
            if(radius>max_radius_) continue;
            line_length=calLineLength(
                             start[0],
                             start[1],
                             end[0],
                             end[1]
                            ); 
            //std::cout<<"line length:"<<line_length<<std::endl;            
            if(line_length>line_max_ || line_length<line_min_) continue;
            // not greedy mode, one is enough
            // for debug
            found =true;
            double x=(start[0]+end[0]+COUNTER_WIDTH*cos(angle))/2;
            double y=(start[1]+end[1]+COUNTER_WIDTH*sin(angle))/2;
            pose2d_.x=x;
            pose2d_.y=y;
            pose2d_.theta=angle;


            Eigen::Quaterniond q;
            q=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ());
            pose_.position.x=filterLowX(x);
            pose_.position.y=filterLowY(y);
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

    double filterLowX(double input){
        

        static float xv[NZEROS+1], yv[NPOLES+1];
        xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
        xv[4] = input / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
        yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
                    + ( -0.1873794924 * yv[0]) + (  1.0546654059 * yv[1])
                    + (  -2.3139884144 * yv[2]) + (  2.3695130072 * yv[3]);
        return yv[4];
    }
    double filterLowY(double input){

        static float xvy[NZEROS+1], yvy[NPOLES+1];
        xvy[0] = xvy[1]; xvy[1] = xvy[2]; xvy[2] = xvy[3]; xvy[3] = xvy[4]; 
        xvy[4] = input / GAIN;
        yvy[0] = yvy[1]; yvy[1] = yvy[2]; yvy[2] = yvy[3]; yvy[3] = yvy[4]; 
        yvy[4] =   (xvy[0] + xvy[4]) + 4 * (xvy[1] + xvy[3]) + 6 * xvy[2]
                    + ( -0.1873794924 * yvy[0]) + (  1.0546654059 * yvy[1])
                    + ( -2.3139884144 * yvy[2]) + (  02.3695130072 * yvy[3]);
        return yvy[4];
    }

    bool recognizeCounter(obj_srv::obj_6d::Request &req,obj_srv::obj_6d::Response &res){
        

        // construct req
        if(req.start){
            geometry_msgs::PoseArray obj_array;
            obj_array.poses.resize(0);
            // if(foud_counter_pose_){
            //     obj_array.poses.push_back(pose_);
            // }
            obj_array.poses.push_back(pose_);
            res.obj_array = obj_array;

        }
        return true;
    }

    static double calLineLength(float x1,float y1,float x2,float y2){
        return (double)sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    }

    static double pi_to_pi(double angle)
    {
        angle = fmod(angle, 2 * M_PI);
        if (angle >= M_PI)
            angle -= 2 * M_PI;
        return angle;
    }

    static double calLineAngle(double x1,double y1,double x2,double y2){
        double slope,angle;
        if (fabs(x1 - x2) > 1e-9)
        {
            slope = (y2 - y1) / (x2 - x1);
            angle = pi_to_pi(atan(slope) + M_PI/2);
        }
        else
        {
            angle = 0.0;
        }
        return angle;
    }

    static double calLineRadius(double x1,double y1,double x2,double y2,double& angle)
    {   
        double radius;
        radius = x1 * cos(angle) + y1 * sin(angle);
        if (radius < 0)
        {
            radius = -radius;
            angle = pi_to_pi(angle + M_PI);
        }
        return radius;
    }

    /*
    [x,y,z]=[ r * cos theta,  r * sin theta , 0]
    */
    void laser2PCloud(const sensor_msgs::LaserScan::ConstPtr& msg){
        std::cout<<" to cloud start\n";
        double angle=angle_min_;
        cloud_->width=range_size_;
        cloud_->height=1;
        cloud_->points.resize(cloud_->width*cloud_->height);
        for(int i=0;i<range_size_;i++){
            angle +=angle_increment_;
            cloud_->points[i].x = msg->ranges[i]*cos(angle);
            cloud_->points[i].y = msg->ranges[i]*sin(angle);
            cloud_->points[i].z = 0;
        }
        std::cout<<" to cloud end\n";
    }

    // Members
    void loadParameters();
    void populateMarkerMsg(const std::vector<gline>&, visualization_msgs::Marker&);
    void cacheData(const sensor_msgs::LaserScan::ConstPtr&);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
};



int main(int argc,char ** argv){
    ros::init(argc, argv, "filter_laser");
    ros::NodeHandle nh;
	ros::NodeHandle nh_local("~");
    double angle_min=-3.14;
    double angle_max=3.14;
    std::string source_file;
    bool init_source;
    nh.param<std::string>("source_file", source_file, "source.pcd");

    //change to true to init source
    nh.param<bool>("init_source", init_source, false);

    double frequency;
    nh.param<double>("frequency", frequency, 20);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);

    FilterLaser FilterLaser(source_file,init_source,0.04,nh,nh_local);


    // init source file
    if(init_source){
        std::cout<<"Start init source."<<std::endl;
        while(ros::ok()){
            rate.sleep();
            ros::spinOnce();
        }
        return 0;
    }
    std::cout<<"Start recognize counter at frequency:"<<frequency<<"Hz\n";
    while (ros::ok())
    {   
        
        ros::spinOnce();
        rate.sleep();
        FilterLaser.runOnce();
    }
    return 0;
}

void FilterLaser::loadParameters(){
    ROS_DEBUG("*************************************");
	ROS_DEBUG("PARAMETERS:");
  
	std::string frame_id, scan_topic;
	bool show_lines;

	nh_local_.param<std::string>("frame_id", frame_id, "laser");
	frame_id_ = frame_id;
	ROS_DEBUG("frame_id: %s", frame_id_.c_str());

	nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
	scan_topic_ = scan_topic;
	ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

	nh_local_.param<bool>("show_lines", show_lines, true);

	pub_markers_ = show_lines;
	ROS_DEBUG("show_lines: %s", show_lines ? "true" : "false");

	// Parameters used by the line extraction algorithm

	int min_line_points,seed_line_points;
	double least_thresh,min_line_length,predict_distance;

	nh_local_.param<double>("least_thresh", least_thresh, 0.03);
	line_feature_.set_least_threshold(least_thresh);
	ROS_DEBUG("least_thresh: %lf", least_thresh);

	nh_local_.param<double>("min_line_length", min_line_length, 0.5);
	line_feature_.set_min_line_length(min_line_length);
	ROS_DEBUG("min_line_length: %lf", min_line_length);
  
	nh_local_.param<double>("predict_distance", predict_distance, 0.02);
	line_feature_.set_predict_distance(predict_distance);
	ROS_DEBUG("predict_distance: %lf", predict_distance);

	nh_local_.param<int>("seed_line_points", seed_line_points, 6);
	line_feature_.set_seed_line_points(seed_line_points);
	ROS_DEBUG("seed_line_points: %d", seed_line_points);

  	nh_local_.param<int>("min_line_points", min_line_points, 20);
  	line_feature_.set_min_line_points(min_line_points);
  	ROS_DEBUG("min_line_points: %d", min_line_points);

  	ROS_DEBUG("*************************************");
}

void FilterLaser::populateMarkerMsg(const std::vector<gline>& lines, visualization_msgs::Marker& marker_msg){
    marker_msg.ns = "line_extraction";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    marker_msg.scale.x = 0.1;
    marker_msg.color.r = 1.0;
    marker_msg.color.g = 0.0;
    marker_msg.color.b = 0.0;
    marker_msg.color.a = 1.0;
    for (std::vector<gline>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
    {
        geometry_msgs::Point p_start;
        p_start.x = cit->x1;
        p_start.y = cit->y1;
        p_start.z = 0;
        marker_msg.points.push_back(p_start);
        geometry_msgs::Point p_end;
        p_end.x = cit->x2;
        p_end.y = cit->y2;
        p_end.z = 0;
        marker_msg.points.push_back(p_end);
    }
    marker_msg.header.frame_id = frame_id_;
    marker_msg.header.stamp = ros::Time::now();
}

void FilterLaser::cacheData(const sensor_msgs::LaserScan::ConstPtr& msg){
    line_feature_.set_angle_increment(angle_increment_);
	line_feature_.set_angle_start(angle_min_);

    std::vector<double> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> index;
    unsigned int i = 0;
	for (double b = msg->angle_min; b <= msg->angle_max; b += msg->angle_increment)
	{
		bearings.push_back(b);
		cos_bearings.push_back(cos(b));
    	sin_bearings.push_back(sin(b));
    	index.push_back(i);
    	i++;
	}

    line_feature_.setCosSinData(bearings, cos_bearings, sin_bearings, index);
	ROS_DEBUG("Data has been cached.");
}