#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "grasp_motion.h"


using namespace std;

bool MPlanning(string planning_group, string reframe, Eigen::Affine3d target_pose)
{
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    move_group.setPoseReferenceFrame(reframe);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPoseTarget(target_pose);
    move_group.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();
    ROS_INFO_NAMED("Visualizing plan RRT (pose goal) %s", success ? "" : "FAILED");
    if (success)
    {
        //ROS_INFO("Visualizing plan RRT (again)");
        move_group.execute(my_plan);
        return 1;
    }
    else
    {
        return 0;
    }
}

int main(int argc, char **argv)
{
    int pick_id = (argc > 1) ? atoi(argv[1]) : 1;
    ros::init(argc, argv, "sda_pick");
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
    tf::TransformListener lr;
    tf::TransformBroadcaster br;


    // publish counter
    std::string package_path = ros::package::getPath("grasp_planning");
    std::string counter_path="file::/"+package_path+"/model/counter.stl";
    Eigen::Affine3d counter_pose=Eigen::Affine3d::Identity();
    Eigen::Matrix3d counter_rot;
    counter_rot=Eigen::AngleAxisd(M_PI*(0.5-0.4843),Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(M_PI*0.5,Eigen::Vector3d::UnitX());
    Eigen::Vector3d counter_trans(1.145,-0.695,0);
    counter_pose.prerotate(counter_rot);
    counter_pose.pretranslate(counter_trans);

    gm.visual_tools_.deleteAllMarkers();
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishMeshBoth("counter",counter_path,counter_pose);
    gm.visual_tools_.trigger();

    moveit_msgs::RobotTrajectory robot_trajectory;

    Eigen::Affine3d pose1=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2=Eigen::Affine3d::Identity();
    Eigen::Matrix3d prot;
    Eigen::Vector3d ptrans;
    prot=Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    ptrans<<1.145-0.18,-0.695+0.1,1.06;
    pose1.prerotate(prot);
    pose1.pretranslate(ptrans);

    pose2=pose1;
    pose2.translate(Eigen::Vector3d(0,-0.2, 0));
    gm.visual_tools_.publishAxisLabeled(pose1, "pose1");
    gm.visual_tools_.trigger();
    gm.visual_tools_.publishAxisLabeled(pose2, "pose2");
    gm.visual_tools_.trigger();

    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*(gm.pm_->robot_state_), display_trajectory.trajectory_start);
    display_trajectory.model_id="motoman_sda5f";
    ros::WallDuration sleep_t(4);

    gm.pm_->robot_state_->setFromIK(gm.pm_->robot_state_->getJointModelGroup(PLANNING_GROUP),pose2);
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
    getchar();
    sleep_t.sleep();

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
    getchar();

    pose2.translate(Eigen::Vector3d(0,-0.1, 0));
    if(!gm.cartesianPlanning(pose2,robot_trajectory)){
        std::cout<<"Cartesian planning failed."<<std::endl;
        ros::shutdown();
        return -1;
    }
    std::cout<<"Cartesian planning: execute ..."<<std::endl;
    gm.pm_->execute(robot_trajectory,false);

    std::cout<<"All completed."<<std::endl;




    


    // ros::WallDuration(1.0).sleep();
    // lr.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
    // lr.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform_camera);
    // client_obj.call(srv_poses);
    getchar();
    ros::shutdown();
    return 0;
}
