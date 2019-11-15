#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "grasp_motion.h"
#include "obj_srv/obj_6d.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>



using namespace std;

int main(int argc, char **argv)
{
    int pick_id = (argc > 1) ? atoi(argv[1]) : 1;
    ros::init(argc, argv, "grasp_based_laser");
    ros::NodeHandle node_handle;
    // ros::AsyncSpinner spinner(4);
    // spinner.start();
    static const std::string PLANNING_GROUP = "arm_right";
    yeebot::PlanningSpec planning_spec;
    Eigen::VectorXi invalid_vector(6);
    invalid_vector<<0,0,0,1,1,0;

    Eigen::Affine3d ref_pose,error_pose;
    Eigen::Vector3d ref_trans;
    Eigen::Matrix3d ref_rot;
    ref_rot=(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ()))* (Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()))*(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q(ref_rot);
    ref_trans<<0.711815,  0.2496, 1.1416;
    ref_pose=Eigen::Affine3d(q);
    ref_pose.pretranslate(ref_trans);
    error_pose=Eigen::Affine3d::Identity();
    error_pose.pretranslate(Eigen::Vector3d(0.045,0,1.0096));
    ref_pose=error_pose.inverse()*ref_pose;

    std::cout<<"reference pose matrix:\n"<<ref_pose.matrix()<<std::endl;
    planning_spec.ref_pose_=ref_pose;
    planning_spec.invalid_vector_=invalid_vector;
    planning_spec.project_error_=1e-3;
    planning_spec.ik_error_=1e-6;



    GraspMotion gm(PLANNING_GROUP,planning_spec);
    Gripper gp;
    //gp.open();
    tf::TransformListener lr;
    tf::TransformBroadcaster br;

    ros::ServiceClient client_counter = node_handle.serviceClient<obj_srv::obj_6d>("/recognize_counter");
    obj_srv::obj_6d srv_pose;
    srv_pose.request.start=true;
    client_counter.call(srv_pose);
    if(srv_pose.response.obj_array.poses.size()<1){
        std::cout<<"Failed to recognize any counter\n";
        return -1;
    }


    // publish counter
    std::string package_path = ros::package::getPath("grasp_planning");
    std::string counter_path="file::/"+package_path+"/model/counter.stl";
    Eigen::Affine3d counter_pose=Eigen::Affine3d::Identity();
    Eigen::Affine3d counter_error=Eigen::Affine3d::Identity();
    Eigen::Matrix3d counter_rot;
    counter_rot=Eigen::AngleAxisd(M_PI*0.5,Eigen::Vector3d::UnitX());
    counter_error.prerotate(counter_rot);



    // copy pose
    Eigen::Affine3d counter2laser=Eigen::Affine3d::Identity();
    Eigen::Affine3d laser2world=Eigen::Affine3d::Identity();

    tf::poseMsgToEigen(srv_pose.response.obj_array.poses[0],counter2laser);
    tf::StampedTransform transform_laser;
    lr.waitForTransform("/world", "/laser", ros::Time(0), ros::Duration(10.0));
    lr.lookupTransform("/world", "/laser", ros::Time(0), transform_laser);
    tf::transformTFToEigen(transform_laser,laser2world);
    laser2world(2,3)=0;// z is 0
    counter_pose=laser2world*counter2laser*counter_error;
    //copy pose end
    Eigen::Affine3d c2gpose=Eigen::Affine3d::Identity();
    Eigen::Matrix3d c2grot;
    c2grot=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    c2gpose.prerotate(c2grot);
    c2gpose.pretranslate(Eigen::Vector3d(-0.45,1.1,0));

    gm.visual_tools_.deleteAllMarkers();
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishMeshBoth("counter",counter_path,counter_pose);
    gm.visual_tools_.trigger();

    moveit_msgs::RobotTrajectory robot_trajectory;

    Eigen::Affine3d pose1=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose3=Eigen::Affine3d::Identity();
    pose3=counter_pose*c2gpose;

    Eigen::Matrix3d prot;
    Eigen::Vector3d ptrans;
    prot=Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    ptrans<<1.145-0.18-0.40,-0.695+0.1,1.10;
    pose1.prerotate(prot);
    pose1.pretranslate(ptrans);

    pose2=pose1;
    pose2.translate(Eigen::Vector3d(0,-0.2, 0));
    gm.visual_tools_.publishAxisLabeled(pose1, "pose1");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose2, "pose2");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose3, "pose3");
    gm.visual_tools_.trigger();

    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*(gm.pm_->robot_state_), display_trajectory.trajectory_start);
    display_trajectory.model_id="motoman_sda5f";
    ros::WallDuration sleep_t(4);

    gm.pm_->robot_state_->setFromIK(gm.pm_->robot_state_->getJointModelGroup(PLANNING_GROUP),pose3);
    Eigen::VectorXd jnv2(gm.dim_);
    gm.pm_->robot_state_->copyJointGroupPositions(PLANNING_GROUP,jnv2);
    gm.pm_->updateRobotState();
    if(!gm.jointPlanningYeebot(jnv2,robot_trajectory)){
        std::cout<<"Joint planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    std::cout<<"Joint planning: execute ..."<<std::endl;

    gm.pm_->execute(robot_trajectory,false);
    std::cout<<"Execution finished.";
    sleep_t.sleep();
    getchar();

    // gm.pm_->updateRobotState();
    // if(!gm.posePlanning(pose2,robot_trajectory)){
    //     std::cout<<"Pose planning failed."<<std::endl;
    //     ros::shutdown();
    //     return -1;
    // }
    // std::cout<<"Pose planning: execute ..."<<std::endl;
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // my_plan.trajectory_=robot_trajectory;
    // gm.pm_->move_group_->execute(my_plan);
    // gm.pm_->execute(robot_trajectory,false);
    // sleep_t.sleep();

    if(!gm.cartesianPlanning(pose1,robot_trajectory)){
        std::cout<<"Cartesian planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    std::cout<<"Cartesian planning: execute ..."<<std::endl;
    gm.pm_->execute(robot_trajectory,false);
    sleep_t.sleep();
    
    gp.close();

    // pose2.translate(Eigen::Vector3d(0,-0.1, 0));
    // if(!gm.cartesianPlanning(pose2,robot_trajectory)){
    //     std::cout<<"Cartesian planning failed."<<std::endl;
    //     ros::shutdown();
    //     return -1;
    // }
    // std::cout<<"Cartesian planning: execute ..."<<std::endl;
    // gm.pm_->execute(robot_trajectory,false);
    // sleep_t.sleep();
    // sleep_t.sleep();
    //start2

    std::cout<<"All completed."<<std::endl;




    


    // ros::WallDuration(1.0).sleep();
    // lr.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
    // lr.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform_camera);
    // client_obj.call(srv_poses);
    //getchar();
    ros::shutdown();
    return 0;
}
