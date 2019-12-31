#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "grasp_motion.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

//using namespace std;

#define height 1.08

/*
grab the box one step by one step
*/

int main(int argc, char **argv)
{
    int pick_id = (argc > 1) ? atoi(argv[1]) : 1;
    ros::init(argc, argv, "grasp_box");
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
    ref_rot=(Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ()));
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

    ros::Time time_ik_start,time_ik_end;




    GraspMotion gm(PLANNING_GROUP,planning_spec);
    Gripper gp;
    gp.open();


    ros::ServiceClient client_counter = node_handle.serviceClient<obj_srv::obj_6d>("/recognize_counter");



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

    time_ik_start=ros::Time::now();
    if(!gm.callTransformation(client_counter,counter2laser)){
        std::cout<<"Failed to recognize any pose\n";
        ros::shutdown();
        return -1;
    }
    gm.getTransformation(laser2world,"/laser");
    time_ik_end=ros::Time::now();
    std::cout<<"counter planning time:"<<time_ik_end-time_ik_start<<std::endl;
    laser2world(2,3)=0;// z is 0
    counter_pose=laser2world*counter2laser*counter_error;
    std::cout<<"counter pose:"<<counter_pose.matrix()<<std::endl;
    //copy pose end
    Eigen::Affine3d c2gpose=Eigen::Affine3d::Identity();
    Eigen::Matrix3d c2grot;
    c2grot=Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    c2gpose.prerotate(c2grot);
    c2gpose.pretranslate(Eigen::Vector3d(-0.30,height,0));
    gm.visual_tools_.delCube(1);
    gm.visual_tools_.delCube(2);
    gm.visual_tools_.deleteAllMarkers();
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishMeshBoth("counter",counter_path,counter_pose);
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishCube(1,  0.6,-0.15,0.35,  0.5,0.5,0.7);
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishCube(2, counter_pose, Eigen::Vector3d(0.31, 2.0, 0.71));
    gm.visual_tools_.trigger();
    gm.visual_tools_.prompt("next");


    moveit_msgs::RobotTrajectory robot_trajectory;

    Eigen::Affine3d pose1=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose3=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose4=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose5=Eigen::Affine3d::Identity();
    Eigen::Affine3d p_place=Eigen::Affine3d::Identity();

    Eigen::VectorXd jnv3(gm.dim_),jnv4(gm.dim_),jnv5(gm.dim_),jnv_place(gm.dim_),jnv_home(gm.dim_);
    jnv_home<<1.6577926874160767, -1.54693603515625, 0.2813563644886017, -0.2986098527908325, 0.09369432926177979, -1.2773751020431519, 0.4743640720844269;
    gm.robot_state_.setJointGroupPositions(gm.pm_->group_name_,jnv_home);
    const Eigen::Affine3d p_init=gm.robot_state_.getGlobalLinkTransform("arm_right_link_gripper");
    
    pose3=counter_pose*c2gpose;
    c2gpose(0,3)=-0.1;
    pose4=counter_pose*c2gpose;
    c2gpose(0,3)=-0.4;    
    pose5=counter_pose*c2gpose;



    Eigen::Matrix3d prot,prot1;
    Eigen::Vector3d ptrans;
    prot=Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    ptrans<<1.145-0.18-0.45,-0.695+0.1,1.10;
    pose1.prerotate(prot);
    pose1.pretranslate(ptrans);

    prot1=Eigen::AngleAxisd(M_PI/6.0,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    p_place.prerotate(prot1);
    p_place.pretranslate(Eigen::Vector3d(0.9,0,height-0.1));    

    pose2=pose1;
    pose2.translate(Eigen::Vector3d(0,-0.2, 0));

   
    // gm.visual_tools_.publishAxisLabeled(pose1, "pose1");
    // gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(p_init, "start");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose3, "pose3");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose4, "pose4");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose5, "pose5");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(p_place, "target");
    gm.visual_tools_.trigger();

    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*(gm.pm_->robot_state_), display_trajectory.trajectory_start);
    display_trajectory.model_id="motoman_sda5f";
    ros::WallDuration sleep_t(2);

    
    gm.pm_->updateRobotState();
    gm.updateRobotState();
    if(!gm.solveIK(pose3,jnv3,2)){
        std::cout<<"Not valid ik for pose3\n";
    }
    if(!gm.solveIK(pose5,jnv5,jnv3,2)){
        std::cout<<"Not valid ik for pose5\n";
    }
    if(!gm.solveIK(p_place,jnv_place,jnv3,2)){
        std::cout<<"Not valid ik for p_place\n";
    }
    gm.updateRobotState();

    std::cout<<"jnv3:"<<jnv3.transpose()<<std::endl;
    std::cout<<"jnv5:"<<jnv3.transpose()<<std::endl;
    std::cout<<"jnv_place:"<<jnv_place.transpose()<<std::endl;

    gm.backHome(jnv_home);
    sleep_t.sleep();
    gm.visual_tools_.prompt("next");
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    time_ik_start=ros::Time::now();
    if(!gm.jointPlanningYeebot(jnv3,robot_trajectory)){
        std::cout<<"Joint planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    time_ik_end=ros::Time::now();
    std::cout<<"planning time:"<<time_ik_end-time_ik_start<<std::endl;
    std::cout<<"Joint planning: execute ..."<<std::endl;
    gm.publishTrajectoryLine(robot_trajectory,gm.visual_tools_,rviz_visual_tools::GREY);
    time_ik_start=ros::Time::now();
    gm.execute2(robot_trajectory,true);
    time_ik_end=ros::Time::now();
    std::cout<<"execution time:"<<time_ik_end-time_ik_start<<std::endl;
    sleep_t.sleep();//not so fast otherwise move_group can not update the joint values at time
    gm.visual_tools_.trigger();
    gm.visual_tools_.prompt("next");
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    time_ik_start=ros::Time::now();
    if(!gm.cartesianPlanning(pose4,robot_trajectory)){
        std::cout<<"Cartesian planning p4 failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    time_ik_end=ros::Time::now();
    std::cout<<"planning time:"<<time_ik_end-time_ik_start<<std::endl;
    std::cout<<"Cartesian planning p4: execute ..."<<std::endl; 
    gm.publishTrajectoryLine(robot_trajectory,gm.visual_tools_,rviz_visual_tools::GREY);
    time_ik_start=ros::Time::now();
    gm.execute2(robot_trajectory,true);
    time_ik_end=ros::Time::now();
    std::cout<<"execution time:"<<time_ik_end-time_ik_start<<std::endl;
    gm.visual_tools_.trigger();
    gm.visual_tools_.prompt("next");
    gp.close();
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    time_ik_start=ros::Time::now();
    if(!gm.cartesianPlanning(pose5,robot_trajectory)){
        std::cout<<"Cartesian planning p5 failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    time_ik_end=ros::Time::now();
    std::cout<<"planning time:"<<time_ik_end-time_ik_start<<std::endl;
    std::cout<<"Cartesian planning p5: execute ..."<<std::endl;    
    gm.publishTrajectoryLine(robot_trajectory,gm.visual_tools_,rviz_visual_tools::GREY);

    time_ik_start=ros::Time::now();
    gm.execute2(robot_trajectory,true);
    time_ik_end=ros::Time::now();
    std::cout<<"execution time:"<<time_ik_end-time_ik_start<<std::endl;
    //gm.visual_tools_.publishCube(2, counter_pose, Eigen::Vector3d(0.31, 3.0, 0.71));
    gm.visual_tools_.trigger();
    gm.visual_tools_.prompt("next");
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    time_ik_start=ros::Time::now();
    if(!gm.jointPlanningConstraint(jnv_place,robot_trajectory)){
        std::cout<<"Constraint planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    // if(!gm.cartesianPlanning(p_place,robot_trajectory)){
    //     std::cout<<"Cartesian planning p5 failed."<<std::endl;
    //     ros::shutdown();
    //     return -1;
    // }
    time_ik_end=ros::Time::now();
    std::cout<<"constraint planning time:"<<time_ik_end-time_ik_start<<std::endl;
    gm.publishTrajectoryLine(robot_trajectory,gm.visual_tools_,rviz_visual_tools::RED);
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    std::cout<<"Constraint planning: execute ..."<<std::endl;
    gm.visual_tools_.trigger();
    gm.visual_tools_.prompt("next");
    time_ik_start=ros::Time::now();
    gm.execute2(robot_trajectory,true);
    time_ik_end=ros::Time::now();
    std::cout<<"execution time:"<<time_ik_end-time_ik_start<<std::endl;
    gp.open();
    sleep_t.sleep();
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    gm.visual_tools_.trigger();
    gm.visual_tools_.prompt("next");//its feasible


    
    // back home
    // gm.pc_normal_-> setStartAndGoalStates(jnv_place,jnv_home);
    // if(!gm.pc_normal_->plan(5.0)){  
    //     std::cout<<"Back home failed."<<std::endl;
    //     ros::shutdown();
    //     return -1;
    // }
    // gm.pc_normal_->getTrajectoryMsg(robot_trajectory);
    // gm.execute2(robot_trajectory,true);
    // std::cout<<"All completed\n";
    time_ik_start=ros::Time::now();
    if(!gm.jointPlanningYeebot(jnv_home,robot_trajectory)){
        std::cout<<"Joint planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    time_ik_end=ros::Time::now();
    std::cout<<"planning time:"<<time_ik_end-time_ik_start<<std::endl;
    gm.publishTrajectoryLine(robot_trajectory,gm.visual_tools_,rviz_visual_tools::GREY);
    std::cout<<"Joint planning: execute ..."<<std::endl;
    time_ik_start=ros::Time::now();
    gm.execute2(robot_trajectory,true);
    time_ik_end=ros::Time::now();
    std::cout<<"execution time:"<<time_ik_end-time_ik_start<<std::endl;
    std::cout<<"completed\n";
 


    // ros::WallDuration(1.0).sleep();
    // lr.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
    // lr.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform_camera);
    //getchar();
    ros::shutdown();
    return 0;
}
