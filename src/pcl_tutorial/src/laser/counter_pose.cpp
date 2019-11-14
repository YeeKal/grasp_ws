#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/rate.h>
#include <sensor_msgs/LaserScan.h> 
#include <tf/transform_broadcaster.h>

#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"

#define PI 3.1415926
#define COUNTER_WIDTH 0.31

class CounterPose{
public:
    ros::NodeHandle nh_;
    pthread_mutex_t mutex_;
    double max_radius_;    //the max circle to locate the counter 
    double line_min_;       // the line length should be during (line_min, line_max_)
    double line_max_;
    std::vector<double > line_pts_; //x1 y1 x2 y2
    tf::TransformBroadcaster broadcaster_;

    bool is_first_;

    ros::Subscriber sub_;

    CounterPose(double max_radius,double line_min,double line_max,std::string topic="/line_segments")
    :max_radius_(max_radius),
     line_min_(line_min),
     line_max_(line_max)
    {
        line_pts_.resize(4);
        pthread_mutex_init(&mutex_,NULL);
        sub_=nh_.subscribe(topic,100,&CounterPose::lineExtracCB,this);

    }
    ~CounterPose(){}

    void lineExtracCB(const laser_line_extraction::LineSegmentList::ConstPtr& msg){
        processData(msg);
    }
    void processData(const laser_line_extraction::LineSegmentList::ConstPtr& msg){
        // find a valid line
        double line_length=0;
        bool found=false;
        double angle=0;
        for(int i=0;i<msg->line_segments.size();i++){
            
            angle=msg->line_segments[i].angle;
            if(angle>PI/2 || angle<-PI/2) continue;
            if(msg->line_segments[i].radius>max_radius_) continue;
            line_length=calLineLength(
                             msg->line_segments[i].start[0],
                             msg->line_segments[i].start[1],
                             msg->line_segments[i].end[0],
                             msg->line_segments[i].end[1]
                            ); 
            if(line_length>line_max_ || line_length<line_min_) continue;
            // not greedy mode, one is enough
            // for debug
            //std::cout<<"line length:"<<line_length<<std::endl;
            found =true;
            line_pts_[0]=msg->line_segments[i].start[0];
            line_pts_[1]=msg->line_segments[i].start[1];
            line_pts_[2]=msg->line_segments[i].end[0];
            line_pts_[3]=msg->line_segments[i].end[1];

            Eigen::Vector3d center((line_pts_[0]+line_pts_[2]+COUNTER_WIDTH*cos(angle))/2,
                                   (line_pts_[1]+line_pts_[3]+COUNTER_WIDTH*sin(angle))/2,
                                   0);
            Eigen::Quaterniond q;
            q=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ());
            broadcaster_.sendTransform(
                tf::StampedTransform(
                tf::Transform(tf::Quaternion(q.x(),q.y(),q.z(),q.w()),tf::Vector3(center(0),center(1),center(2))),
                ros::Time::now(),"laser","counter"));
            
            break;
        }
        


    }

    static double calLineLength(float x1,float y1,float x2,float y2){
        return (double)sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    }
};

int main(int argc,char ** argv){
    ros::init(argc, argv, "counter_pose");
    ros::NodeHandle nh;
    double max_radius=2.0;
    double line_min=-3.14;
    double line_max=3.14;
    if(argc>3){
        max_radius=std::stod(argv[1]);
        line_min=std::stod(argv[2]);
        line_max=std::stod(argv[3]);
    }
    ros::Rate rate(1);
    CounterPose counter(max_radius,line_min,line_max);

    ros::spin();
    return 0;
}