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

    ros::Subscriber sub_;
    ros::Publisher pub_;

    Laser(int fre=20,std::string topic="/scan")
    :rate_(ros::Rate(fre)){
        pthread_mutex_init(&mutex_,NULL);
        sub_=nh_.subscribe(topic,100,&Laser::laserCB,this);
        pub_=nh_.advertise<sensor_msgs::LaserScan>("scan2",100);
        ros::spin();

    }
    ~Laser(){}

    void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
        //TODO: add thread mutex
        pthread_mutex_lock(&mutex_);
        data_raw_=*msg;//->data;
        pthread_mutex_unlock(&mutex_);
        std::cout
    << "----------------------------------\n"
    << "angle_min:"<<data_raw_.angle_min<<"\n"
    << "angle_max:"<<data_raw_.angle_max<<"\n"
    << "angle_increment:"<<data_raw_.angle_increment<<"\n"
    << "time_increment:"<<data_raw_.time_increment<<"\n"
    << "step size:"<<(int)((data_raw_.angle_max-data_raw_.angle_min)/data_raw_.angle_increment)<<"\n"
    << "ranges size:"<<data_raw_.ranges.size()<<"\n"
    << "------------------------------------\n";
        rate_.sleep();
    }
    void getData(){
        data_cur_=data_raw_;
    }
};

int main(int argc,char ** argv){
    ros::init(argc, argv, "read_laser");
    ros::NodeHandle nh;
    if(argc>1){
        std::cout<<"you input:"<<argv[1]<<std::endl;
    }
    ros::Rate rate(1);
    rate.sleep();
    rate.sleep();
    Laser laser;
    laser.getData();
    std::cout
    << "----------------------------------\n"
    << "angle_min:"<<laser.data_cur_.angle_min<<"\n"
    << "angle_max:"<<laser.data_cur_.angle_max<<"\n"
    << "angle_increment:"<<laser.data_cur_.angle_increment<<"\n"
    << "time_increment:"<<laser.data_cur_.time_increment<<"\n"
    << "step size:"<<(int)((laser.data_cur_.angle_max-laser.data_cur_.angle_min)/laser.data_cur_.angle_increment)<<"\n"
    << "ranges size:"<<laser.data_cur_.ranges.size()<<"\n"
    << "------------------------------------\n";

    return 0;
}