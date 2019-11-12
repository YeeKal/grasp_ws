#ifndef GRASP_MOTION_H
#define GRASP_MOTION_H

#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <fstream>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <yeebot_core/planning_context.h>
#include <yeebot_core/planning_manager.h>
#include <yeebot_core/robot_visual_tools.h>
#include <yeebot_commute/JointInfo.h>
#include <yeebot_core/cbirrt.h>

#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <motoman_msgs/WriteSingleIO.h>
#include "obj_srv/obj_6d.h"

// TODO: split 'grouping name' from 'PlanningManager'
//       the planning_scene/robot_state/robot_model should be the same in different planning group

std::string IO_SERVICE_NAME="write_single_io";
std::string BASE_LINK="world";

class Gripper{
public:
    ros::NodeHandle nh_;
    motoman_msgs::WriteSingleIO srv_io_open_;
    motoman_msgs::WriteSingleIO srv_io_close_;
    ros::ServiceClient client_gripper_;

    Gripper(ros::NodeHandle node_handle=ros::NodeHandle())
    :nh_(node_handle),
    client_gripper_(node_handle.serviceClient<motoman_msgs::WriteSingleIO>(IO_SERVICE_NAME))
    {
        srv_io_open_.request.address = 10170;
        srv_io_open_.request.value = 0;
        srv_io_close_.request.address = 10170;
        srv_io_close_.request.value = 1;
    }
    ~Gripper(){}

    void open(){client_gripper_.call(srv_io_open_);}

    void close(){client_gripper_.call(srv_io_close_);}

};



class GraspMotion{
public:
    yeebot::PlanningManagerPtr pm_;
    yeebot::PlanningContextPtr pc_normal_,pc_constraint_;
    yeebot::RobotVisualTools visual_tools_;
    ros::NodeHandle nh_;
    int dim_;

    double time_plan_normal_;
    double time_plan_project_;
    double seg_factor_;
    double delta_factor_;

    GraspMotion(std::string group_name,yeebot::PlanningSpec planning_spec,ros::NodeHandle node_handle=ros::NodeHandle())
    :pm_(new yeebot::PlanningManager(group_name,true)),
    pc_normal_(new yeebot::PlanningContext(planning_spec,pm_,yeebot::PlanType::NORMAL)),
    pc_constraint_(new yeebot::PlanningContext(planning_spec,pm_,yeebot::PlanType::AXIS_PROJECT)),
    nh_(node_handle),
    visual_tools_(BASE_LINK,pm_->planning_scene_,pm_->robot_state_),
    dim_(pc_normal_->space_->getDimension()),
    time_plan_normal_(5),time_plan_project_(10),
    seg_factor_(0.2),delta_factor_(0.02)
    {
        // pm_->execute_action_client_->waitForServer();
        // if(! pm_->execute_action_client_->isServerConnected()){
        //     std::cout<<"server is not connected.\n";
        // }
        pm_->move_group_->setPlannerId("RRTConnectkConfigDefault");
        pm_->updateRobotState();

        // set planning parameter
        pc_normal_->space_->setLongestValidSegmentFraction(0.005);
        pc_normal_->planner_->as<ompl::geometric::RRTConnect>()->setRange(pc_normal_->space_->getMaximumExtent()*seg_factor_);
        pc_constraint_->planner_->as<ompl::geometric::CBIRRT>()->setRange(pc_constraint_->space_->getMaximumExtent()*seg_factor_);
        pc_constraint_->space_->as<ompl::base::YeeProjectedStateSpace>()->setDelta(delta_factor_*pc_constraint_->space_->getMaximumExtent());
        pc_constraint_->space_->as<ompl::base::YeeProjectedStateSpace>()->setMaxStep(pc_constraint_->space_->getMaximumExtent()*seg_factor_);
    }
    ~GraspMotion(){}

    // 
    bool posePlanning(Eigen::Affine3d target_pose,moveit_msgs::RobotTrajectory& robot_trajectory,std::string reframe=BASE_LINK){
        //pm_->move_group_->setPoseReferenceFrame(reframe);
        pm_->move_group_->setPoseTarget(target_pose);
        //pm_->move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (pm_->move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!success){
            return false;
        }
        std::cout<<"pose planning ...\n";
        robot_trajectory=my_plan.trajectory_;
        return true;
    }

    bool posePlanningYeebot(Eigen::Affine3d target_pose,moveit_msgs::RobotTrajectory& robot_trajectory,std::string reframe=BASE_LINK){
        pm_->robot_state_->setFromIK(pm_->robot_state_->getJointModelGroup(pm_->group_name_),target_pose);
        Eigen::VectorXd jnv2(dim_);
        pm_->robot_state_->copyJointGroupPositions(pm_->group_name_,jnv2);

        return jointPlanningYeebot(jnv2,robot_trajectory);
    }

    // move to a single point in Cartesian path
    bool cartesianPlanning(Eigen::Affine3d target_pose,moveit_msgs::RobotTrajectory& robot_trajectory,std::string reframe=BASE_LINK){
        geometry_msgs::Pose pose_msg;
        std::vector<geometry_msgs::Pose> waypoints;
        tf::poseEigenToMsg(target_pose,pose_msg);
        waypoints.push_back(pose_msg);

        pm_->move_group_->setPoseReferenceFrame(reframe);
        pm_->move_group_->setStartStateToCurrentState();
        const double jump_threshold = 0.0; //0.0
        const double eef_step = 0.005;
        double fraction = pm_->move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, robot_trajectory);
        std::cout<<"Cartesian Planning:"<<fraction * 100.0<<"acheived"<<std::endl;
        if (fraction < 1.0)
        {
            return false;
        }
        return true;
    }
    
    // move along a series of points in Cartesian path
    bool cartesianPlanning(EigenSTL::vector_Affine3d& target_poses,moveit_msgs::RobotTrajectory& robot_trajectory,std::string reframe=BASE_LINK){
        std::vector<geometry_msgs::Pose> waypoints;
        for(int i=0;i<target_poses.size();i++){
            geometry_msgs::Pose pose_msg;
            tf::poseEigenToMsg(target_poses[i],pose_msg);
            waypoints.push_back(pose_msg);
        }
        
        pm_->move_group_->setPoseReferenceFrame(reframe);
        pm_->move_group_->setStartStateToCurrentState();
        const double jump_threshold = 0.0; //0.0
        const double eef_step = 0.005;
        double fraction = pm_->move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, robot_trajectory);
        std::cout<<"Cartesian Planning:"<<fraction * 100.0<<"acheived"<<std::endl;
        if (fraction < 1.0)
        {
            return false;
        }
        return true;
    }

    // joint space planning by Moveit
    bool jointPlanningMoveit(Eigen::VectorXd& jnv,moveit_msgs::RobotTrajectory& robot_trajectory){
        return pm_->moveitPlan(jnv,robot_trajectory);
    }

    // joint space planning by yeebot
    bool jointPlanningYeebot(Eigen::VectorXd& jnv,moveit_msgs::RobotTrajectory& robot_trajectory){
        Eigen::VectorXd jnv_cur(dim_);
        pm_->robot_state_->copyJointGroupPositions(pm_->group_name_,jnv_cur);
        pc_normal_-> setStartAndGoalStates(jnv_cur,jnv);
        if(!pc_normal_->plan(time_plan_normal_)){  
            return false;
        }
        pc_normal_->getTrajectoryMsg(robot_trajectory);
        return true;
    }

    // joint space constraint planning by yeebot
    bool jointPlanningConstraint(Eigen::VectorXd& jnv,moveit_msgs::RobotTrajectory& robot_trajectory){
        Eigen::VectorXd jnv_cur(dim_);
        pm_->robot_state_->copyJointGroupPositions(pm_->group_name_,jnv_cur);
        pc_constraint_->setStartAndGoalStates(jnv_cur,jnv);
        //time_ik_start=ros::Time::now();
        if(!pc_constraint_->plan(time_plan_project_)){
            return false;
        }
        pc_constraint_->getTrajectoryMsg(robot_trajectory);
        return true;
    }

};

#endif