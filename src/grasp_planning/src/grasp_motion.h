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
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
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
    std::unique_ptr<moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle> execute_trajectory_handle_;
    std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> > ac_;
    tf::TransformListener lr_;
    tf::TransformBroadcaster br_;
    obj_srv::obj_6d srv_pose_;

    robot_state::RobotState robot_state_;


    double time_plan_normal_;
    double time_plan_project_;
    double seg_factor_;
    double delta_factor_;

    GraspMotion(std::string group_name,yeebot::PlanningSpec planning_spec,ros::NodeHandle node_handle=ros::NodeHandle())
    :pm_(new yeebot::PlanningManager(group_name,true)),
    robot_state_(*(pm_->robot_state_)),
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
        initExecuteHandle();
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

    void initExecuteHandle(){
         std::string action_name="sda5f/sda5f_r2_controller";//"fake_arm_left_controller";//"sda5f/sda5f_r1_controller";//for the left arm.r2 for right arm
        std::string action_ns="joint_trajectory_action";
        execute_trajectory_handle_.reset(new moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle(action_name,action_ns));
        if(!execute_trajectory_handle_->isConnected()){
            ROS_INFO("Failed to connect server.");
        }
        else{
            ROS_INFO("connect to server.");
        }

        // execute action client
        ac_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/sda5f/sda5f_r2_controller/joint_trajectory_action", true));
        //ac_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("joint_trajectory_action", true));
        ROS_INFO("Wait for execute action to connect server.");
        ac_->waitForServer(); //will wait for infinite time
        ROS_INFO("Action server started, sending goal.");

        // client 
        srv_pose_.request.start=true;
    }

    void getTransformation(Eigen::Affine3d &trans,std::string target,std::string base=BASE_LINK){
        tf::StampedTransform transform_frame;
        lr_.waitForTransform(base, target, ros::Time(0), ros::Duration(10.0));
        lr_.lookupTransform(base, target, ros::Time(0), transform_frame);
        tf::transformTFToEigen(transform_frame,trans);
    }

    bool callTransformation(ros::ServiceClient & client,Eigen::Affine3d &trans){
        client.call(srv_pose_);
        if(srv_pose_.response.obj_array.poses.size()<1){
            return false;
        }
        tf::poseMsgToEigen(srv_pose_.response.obj_array.poses[0],trans);
        return true;
    }

    bool solveIK(const Eigen::Affine3d & eigen_pose,Eigen::VectorXd &joint_values,Eigen::VectorXd &ref_values,int max_attempts=1){
        robot_state_.setJointGroupPositions(pm_->group_name_,ref_values);
        solveIK(eigen_pose,joint_values,max_attempts);
    }
    bool solveIK(const Eigen::Affine3d & eigen_pose,Eigen::VectorXd &joint_values,int max_attempts=1){
        const robot_state::JointModelGroup* jmg = robot_state_.getJointModelGroup(pm_->group_name_);
        for(int i=0;i<max_attempts;i++){
            if(robot_state_.setFromIK(jmg,eigen_pose)){
                robot_state_.copyJointGroupPositions(jmg,joint_values);
                return true;
            }
        }
        return false;
    }

    bool execute(const moveit_msgs::RobotTrajectory& robot_trajectory,bool wait){
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory=robot_trajectory.joint_trajectory;

        ac_->sendGoal(goal);
        if(!wait) return true;
        bool finished_before_timeout = ac_->waitForResult(ros::Duration(30.0));

        if (!finished_before_timeout)   {
            ROS_INFO("Action did not finish before the time out.");
            return false;
        }
        actionlib::SimpleClientGoalState state = ac_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        if(state==actionlib::SimpleClientGoalState::SUCCEEDED){
            return true;
        }
        return false;                                          
    }

    bool execute2(const moveit_msgs::RobotTrajectory& robot_trajectory,bool wait){
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory=robot_trajectory.joint_trajectory;

        ac_->sendGoal(goal);
        if(!wait) return true;
        bool finished_before_timeout = ac_->waitForResult(ros::Duration(30.0));

        if (!finished_before_timeout)   {
            ROS_INFO("Action did not finish before the time out.");
            return false;
        }
        actionlib::SimpleClientGoalState state = ac_->getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
        if(state==actionlib::SimpleClientGoalState::SUCCEEDED){
            return true;
        }
        return false;                                          
    }

    void updateRobotState(){
        Eigen::VectorXd jnv2(dim_);
        pm_->robot_state_->copyJointGroupPositions(pm_->group_name_,jnv2);
        robot_state_.setJointGroupPositions(pm_->group_name_,jnv2);
        robot_state_.update();
    }


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
        //std::cout<<"pose planning ...\n";
        robot_trajectory=my_plan.trajectory_;
        return true;
    }

    bool posePlanningYeebot(Eigen::Affine3d target_pose,moveit_msgs::RobotTrajectory& robot_trajectory,std::string reframe=BASE_LINK){
        updateRobotState();
        robot_state_.setFromIK(robot_state_.getJointModelGroup(pm_->group_name_),target_pose);
        Eigen::VectorXd jnv2(dim_);
        robot_state_.copyJointGroupPositions(pm_->group_name_,jnv2);

        return jointPlanningYeebot(jnv2,robot_trajectory);
    }

    // move to a single point in Cartesian path
    bool cartesianPlanning(Eigen::Affine3d target_pose,moveit_msgs::RobotTrajectory& robot_trajectory,std::string reframe=BASE_LINK){
        geometry_msgs::Pose pose_msg;
        std::vector<geometry_msgs::Pose> waypoints;
        tf::poseEigenToMsg(target_pose,pose_msg);
        waypoints.push_back(pose_msg);

        pm_->move_group_->setPoseReferenceFrame(reframe);
        //pm_->move_group_->setStartStateToCurrentState();
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
        pc_normal_->ss_->clear();
        Eigen::VectorXd jnv_cur(dim_);
        pm_->robot_state_->copyJointGroupPositions(pm_->group_name_,jnv_cur);

        pc_normal_-> setStartAndGoalStates(jnv_cur,jnv);
        
        std::cout<<"jnv_cur:"<<jnv_cur.transpose()<<std::endl;
        if(!pc_normal_->plan(time_plan_normal_)){  
            return false;
        }
        pc_normal_->getTrajectoryMsg(robot_trajectory);
        return true;
    }

    // joint space constraint planning by yeebot
    bool jointPlanningConstraint(Eigen::VectorXd& jnv,moveit_msgs::RobotTrajectory& robot_trajectory){
        pc_constraint_->ss_->clear();
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
    void backHome(Eigen::VectorXd& jnv){
        pm_->updateRobotState();
        moveit_msgs::RobotTrajectory robot_trajectory;
        jointPlanningYeebot(jnv,robot_trajectory);
        execute(robot_trajectory,true);
    }

    void publishTrajectoryLine(moveit_msgs::RobotTrajectory robot_trajectory,rviz_visual_tools::RvizVisualTools &visual_tools,const rviz_visual_tools::colors& color){
        std::vector<geometry_msgs::Point> line;
        for(std::size_t i=0;i<robot_trajectory.joint_trajectory.points.size();i++){
            Eigen::VectorXd jnv(dim_);
            for(int k=0;k<dim_;k++){
                jnv[k]=robot_trajectory.joint_trajectory.points[i].positions[k];
            }
            
            robot_state_.setJointGroupPositions(pm_->group_name_,jnv);
            const Eigen::Affine3d end_pose=robot_state_.getGlobalLinkTransform("arm_right_link_gripper");
            Eigen::Vector3d trans_pose;
            trans_pose<<end_pose(0,3),end_pose(1,3),end_pose(2,3);
            visual_tools.publishSphere(trans_pose,color,rviz_visual_tools::SMALL);
            geometry_msgs::Point point1;
            point1.x=end_pose.translation().x();
            point1.y=end_pose.translation().y();
            point1.z=end_pose.translation().z();

            line.push_back(point1);
        }
        const double radius=0.01;
        visual_tools.publishPath(line,color,radius);
        visual_tools.trigger();
        updateRobotState();
    }

};

#endif