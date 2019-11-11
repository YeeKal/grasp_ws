#ifndef GRASP_MOTION_H
#define GRASP_MOTION_H

#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <ros/ros.h>
#include <fstream>

#include <yeebot_core/planning_context.h>
#include <yeebot_core/planning_manager.h>
#include <yeebot_core/robot_visual_tools.h>
#include <yeebot_commute/JointInfo.h>
#include <yeebot_core/cbirrt.h>

// TODO: split 'grouping name' from 'PlanningManager'
//       the planning_scene/robot_state/robot_model should be the same in different planning group



class GraspMotion{
public:
    yeebot::PlanningManagerPtr pm_;
    yeebot::PlanningContextPtr pc_normal_,pc_constraint_;
    yeebot::RobotVisualTools visual_tools_;

    GraspMotion(std::string group_name,yeebot::PlanningSpec planning_spec)
    :pm_(new yeebot::PlanningManager(group_name,true)),
    pc_normal_(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::NORMAL)),
    pc_constraint_(new yeebot::PlanningContext(planning_spec,pm,yeebot::PlanType::AXIS_PROJECT)),
    visual_tools_("world",pm_->planning_scene_,pm_->robot_state_)
    {
        // set planning parameter
    }
    ~GraspMotion(){}

};

#endif