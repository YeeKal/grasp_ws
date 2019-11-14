#include <iostream>
#include <vector>
#include <string>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/rate.h>
#include <sensor_msgs/LaserScan.h> 

class Laser{
public:
    ros::NodeHandle nh_;
    sensor_msgs::LaserScan data_raw_;
    sensor_msgs::LaserScan data_cur_;
    pthread_mutex_t mutex_;
    ros::Rate rate_;
    double angle_min_;
    double angle_max_;

    bool is_first_;

    ros::Subscriber sub_;
    ros::Publisher pub_;

    Laser(double angle_min=-3.14,double angle_max=3.14,int fre=20,std::string topic="/scan")
    :angle_min_(angle_min),
    angle_max_(angle_max),
    is_first_(true),
    rate_(ros::Rate(fre)){
        pthread_mutex_init(&mutex_,NULL);
        sub_=nh_.subscribe(topic,100,&Laser::laserCB,this);
        pub_=nh_.advertise<sensor_msgs::LaserScan>("scan2",100);

    }
    ~Laser(){}

    void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        //TODO: add thread mutex
    //     pthread_mutex_lock(&mutex_);
    //     data_raw_=*msg;//->data;
    //     pthread_mutex_unlock(&mutex_);
    //     std::cout
    // << "----------------------------------\n"
    // << "angle_min:"<<data_raw_.angle_min<<"\n"
    // << "angle_max:"<<data_raw_.angle_max<<"\n"
    // << "angle_increment:"<<data_raw_.angle_increment<<"\n"
    // << "time_increment:"<<data_raw_.time_increment<<"\n"
    // << "step size:"<<(int)((data_raw_.angle_max-data_raw_.angle_min)/data_raw_.angle_increment)<<"\n"
    // << "ranges size:"<<data_raw_.ranges.size()<<"\n"
    // << "------------------------------------\n";
        //rate_.sleep();
        processData(msg);
    }
    void processData(const sensor_msgs::LaserScan::ConstPtr& msg){
        //cut by angle range
        if(is_first_){
            if(angle_min_<msg->angle_min) angle_min_=msg->angle_min;
            if(angle_max_>msg->angle_max) angle_max_=msg->angle_max;
            data_raw_.angle_min=angle_min_;
            data_raw_.angle_max=angle_max_;
            data_raw_.angle_increment=msg->angle_increment;
            data_raw_.time_increment=msg->time_increment;
            is_first_=false;
        }
        data_raw_.ranges.clear();
        data_raw_.intensities.clear();
        data_raw_.header.frame_id="laser";
        data_raw_.header.stamp=ros::Time();
        data_raw_.scan_time=msg->scan_time;
        data_raw_.range_min=msg->range_min;
        data_raw_.range_max=msg->range_max;
        int start=(int)((angle_min_-msg->angle_min)/data_raw_.angle_increment);
        int end=(int)((angle_max_-msg->angle_min)/data_raw_.angle_increment);
        for(int i=start;i<end;i++){
            data_raw_.ranges.push_back(msg->ranges[i]);
            data_raw_.intensities.push_back(msg->intensities[i]);
        }
        pub_.publish(data_raw_);


    }
    void getData(){
        data_cur_=data_raw_;
    }
};

int main(int argc,char ** argv){
    ros::init(argc, argv, "read_laser");
    ros::NodeHandle nh;
    double angle_min=-3.14;
    double angle_max=3.14;
    if(argc>2){
        angle_min=std::stod(argv[1]);
        angle_max=std::stod(argv[2]);
    }
    ros::Rate rate(1);
    rate.sleep();
    rate.sleep();
    Laser laser(angle_min,angle_max);
    laser.getData();


    ros::spin();
    return 0;
}