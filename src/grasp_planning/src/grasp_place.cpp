#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "grasp_motion.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

//using namespace std;

#define height 1.08
/*
place the box
*/

int main(int argc, char **argv)
{
    int pick_id = (argc > 1) ? atoi(argv[1]) : 1;
    ros::init(argc, argv, "grasp_place");
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
    planning_spec.project_error_=1e-2;
    planning_spec.ik_error_=1e-6;

    ros::Time time_ik_start,time_ik_end;




    GraspMotion gm(PLANNING_GROUP,planning_spec);
    Gripper gp;
    gp.open();
    gm.visual_tools_.removeAll();
    gm.visual_tools_.trigger();



    ros::ServiceClient client_counter = node_handle.serviceClient<obj_srv::obj_6d>("/recognize_counter");
    ros::ServiceClient client_box = node_handle.serviceClient<obj_srv::obj_6d>("/recognize_block");




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
    Eigen::Affine3d box2camera=Eigen::Affine3d::Identity();
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
    Eigen::Affine3d pose1_down=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2_down=Eigen::Affine3d::Identity();

    Eigen::Affine3d pose3=Eigen::Affine3d::Identity();// start into 
    Eigen::Affine3d pose4=Eigen::Affine3d::Identity();// grasp pose
    Eigen::Affine3d pose4_up=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose5=Eigen::Affine3d::Identity();//back pose
    Eigen::Affine3d pose5_up=Eigen::Affine3d::Identity();//back pose up
    Eigen::Affine3d p_place=Eigen::Affine3d::Identity();
    Eigen::Affine3d box_pose=Eigen::Affine3d::Identity();


    Eigen::VectorXd jnv3(gm.dim_),jnv4(gm.dim_),jnv5(gm.dim_),jnv_place(gm.dim_),jnv_home(gm.dim_),jnv2(gm.dim_),jnv1(gm.dim_);
    jnv_home<<1.6577926874160767, -1.54693603515625, 0.2813563644886017, -0.2986098527908325, 0.09369432926177979, -1.2773751020431519, 0.4743640720844269;
    gm.robot_state_.setJointGroupPositions(gm.pm_->group_name_,jnv_home);
    const Eigen::Affine3d p_init=gm.robot_state_.getGlobalLinkTransform("arm_right_link_gripper");
    
    pose3=counter_pose*c2gpose;
    c2gpose(0,3)=-0.1+0.05;
    pose4=counter_pose*c2gpose;
    c2gpose(0,3)=-0.38;    
    pose5=counter_pose*c2gpose;
    c2gpose(0,3)=0.05;
    box_pose=counter_pose*c2gpose;

    c2gpose(0,3)=-0.1+0.05;
    c2gpose(1,3)=c2gpose(1,3)+0.05;
    pose4_up=counter_pose*c2gpose;
    c2gpose(0,3)=-0.38;    
    pose5_up=counter_pose*c2gpose;

    c2gpose(1,3)=c2gpose(1,3)+0.35+0.01;
    c2gpose(0,3)=-0.38;  
    pose2= counter_pose*c2gpose;
    c2gpose(0,3)=-0.1+0.05;  
    pose1=counter_pose*c2gpose;
    c2gpose(1,3)=c2gpose(1,3)-0.06;
    pose1_down=counter_pose*c2gpose;
    c2gpose(0,3)=-0.38;  
    pose2_down= counter_pose*c2gpose;


    Eigen::Matrix3d prot1;

    prot1=Eigen::AngleAxisd(M_PI/6.0,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    p_place.prerotate(prot1);
    p_place.pretranslate(Eigen::Vector3d(0.9,0,height-0.1));    


   
    gm.visual_tools_.publishAxisLabeled(pose1, "pose1");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose2, "pose2");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose1_down, "pose1_down");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose2_down, "pose2_down");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(p_init, "start");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose3, "pose3");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose4, "pose4");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose4_up, "pose4_up");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose5, "pose5");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose5_up, "pose5_up");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(p_place, "target");
    gm.visual_tools_.trigger();

    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*(gm.pm_->robot_state_), display_trajectory.trajectory_start);
    display_trajectory.model_id="motoman_sda5f";
    ros::WallDuration sleep_t(0.8);

    
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
    if(!gm.solveIK(pose2,jnv2,jnv3,2)){
        std::cout<<"Not valid ik for p_place\n";
    }
    if(!gm.solveIK(pose1,jnv1,jnv2,2)){
        std::cout<<"Not valid ik for p_place\n";
    }
    gm.updateRobotState();

    std::cout<<"jnv3:"<<jnv3.transpose()<<std::endl;
    std::cout<<"jnv5:"<<jnv3.transpose()<<std::endl;
    std::cout<<"jnv_place:"<<jnv_place.transpose()<<std::endl;
    std::cout<<"jnv2:"<<jnv2.transpose()<<std::endl;
    std::cout<<"jnv1:"<<jnv1.transpose()<<std::endl;

    gm.backHome(jnv_home);
    gm.visual_tools_.prompt("next");

    // if open camera
    // if(!gm.callTransformation(client_box,box2camera)){
    //     std::cout<<"Failed to recognize any pose\n";
    //     ros::shutdown();
    //     return -1;
    // }
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    //jnv3
    if(!gm.jointPlanningYeebot(jnv3,robot_trajectory)){
        std::cout<<"Joint planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    sleep_t.sleep();//not so fast otherwise move_group can not update the joint values at time
    gm.pm_->updateRobotState();
    gm.updateRobotState();



    // pose4
    if(!gm.cartesianPlanning(pose4,robot_trajectory)){
        std::cout<<"Cartesian planning p4 failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    gp.close();
    sleep_t.sleep();
    gm.pm_->updateRobotState();
    gm.updateRobotState();

     //pose4_up
    if(!gm.cartesianPlanning(pose4_up,robot_trajectory)){
        std::cout<<"Cartesian planning p4 failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    sleep_t.sleep();
    gm.pm_->updateRobotState();
    gm.updateRobotState();


    //pose5_up
    if(!gm.cartesianPlanning(pose5_up,robot_trajectory)){
        std::cout<<"Cartesian planning p5_up failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    sleep_t.sleep();
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    //pose2
    // jnv2
    // if(!gm.jointPlanningConstraint(jnv2,robot_trajectory)){
    //     std::cout<<"Constraint planning failed."<<std::endl;
    //     ros::shutdown();
    //     return -1;
    // }
    // gm.execute2(robot_trajectory,true);
    // sleep_t.sleep();
    // gm.pm_->updateRobotState();
    // gm.updateRobotState();

    // if(!gm.cartesianPlanning(pose2,robot_trajectory)){
    //     std::cout<<"Cartesian planning p2 failed."<<std::endl;
    //     ros::shutdown();
    //     return -1;
    // }
    // gm.execute2(robot_trajectory,true);
    // sleep_t.sleep();
    // gm.pm_->updateRobotState();
    // gm.updateRobotState();

    //pose1
    // jnv1
    if(!gm.jointPlanningConstraint(jnv1,robot_trajectory)){
        std::cout<<"Constraint planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    sleep_t.sleep();
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    // if(!gm.cartesianPlanning(pose1,robot_trajectory)){
    //     std::cout<<"Cartesian planning p1 failed."<<std::endl;
    //     ros::shutdown();
    //     return -1;
    // }
    // gm.execute2(robot_trajectory,true);
    // sleep_t.sleep();
    // gm.pm_->updateRobotState();
    // gm.updateRobotState();

     //pose1_down
    if(!gm.cartesianPlanning(pose1_down,robot_trajectory)){
        std::cout<<"Cartesian planning p1_down failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    gp.open();
    sleep_t.sleep();
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    //pose2_down
    if(!gm.cartesianPlanning(pose2_down,robot_trajectory)){
        std::cout<<"Cartesian planning p2_down failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    sleep_t.sleep();
    gm.pm_->updateRobotState();
    gm.updateRobotState();

    



    
    // back home
    if(!gm.jointPlanningYeebot(jnv_home,robot_trajectory)){
        std::cout<<"Joint planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    gm.execute2(robot_trajectory,true);
    std::cout<<"completed\n";
    ros::shutdown();
    return 0;
}
