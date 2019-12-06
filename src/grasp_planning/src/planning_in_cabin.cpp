/************************
*move group for ur5
*Author: yee
*Date: 2018-03-31
*************************/

//collision object from stl
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc,char **argv){
    ros::init(argc,argv,"planning_in_cabin");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static std::string PLANNING_GROUP="arm_right";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    bool success;
ros::Publisher execute_trajectory
        =node_handle.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 100, true);
     

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup *joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::cout<<"continuous num:"<<joint_model_group->getContinuousJointModels().size()<<std::endl;
    for(int i=0;i<joint_model_group->getActiveJointModels().size();i++){
        std::cout<<joint_model_group->getActiveJointModelNames()[i].c_str()<<"  ";
        std::cout<<(*(joint_model_group->getActiveJointModelsBounds()[i]))[0].min_position_;
        std::cout<<" "<<(*(joint_model_group->getActiveJointModelsBounds()[i]))[0].max_position_<<std::endl;

    }

    namespace rvt=rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose=Eigen::Isometry3d::Identity();
    text_pose.translation().z()=0.75;
    visual_tools.publishText(text_pose," motion planning", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("next step");

    Eigen::Affine3d pose1=Eigen::Affine3d::Identity();
    Eigen::Affine3d pose2=Eigen::Affine3d::Identity();
    Eigen::Matrix3d prot;
    Eigen::Vector3d ptrans;
    prot=Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitZ());
    ptrans<<1.145-0.18,-0.695+0.1,1.10;
    pose1.prerotate(prot);
    pose1.pretranslate(ptrans);

    pose2=pose1;
    pose2.translate(Eigen::Vector3d(0,-0.2, 0));
    visual_tools.publishAxisLabeled(pose1, "pose1");
    visual_tools.trigger();
    visual_tools.publishAxisLabeled(pose2, "pose2");
    visual_tools.trigger();

    //define pose goal
    // geometry_msgs::Pose start,pose1,pose2;
    // start.orientation.w=1;
    // start.position.x=cabin_x-0.1;
    // start.position.y=cabin_y=0;
    // start.position.z=cabin_z+0.5;

    // pose1.orientation.w= 1.0;
    // pose1.position.x =0.5;
    // pose1.position.y = -0.2;
    // pose1.position.z = 0.2;

    // pose2.orientation.w= 1.0;
    // pose2.position.x=0.1;//0.3;
    // pose2.position.y=0.4;
    // pose2.position.z=0.3;

    move_group.setPoseTarget(pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal plan: %s", success ? "SUCCESSED" : "FAILED"); 
    if(success){
        //visual_tools.publishAxisLabeled(start, "start");//coordinates with 3 axis
        visual_tools.publishText(text_pose,"move to start ", rvt::WHITE,rvt::XLARGE);
        move_group.execute(my_plan);
    }
    visual_tools.trigger();
    visual_tools.prompt("next");//its feasible

    //pose1
    move_group.setPoseTarget(pose2);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Pose goal plan: %s", success ? "SUCCESSED" : "FAILED"); 
    if(success){
        visual_tools.publishText(text_pose,"move to pose1 ", rvt::WHITE,rvt::XLARGE);
        move_group.execute(my_plan);
    }



    //release memory
    std::vector<std::string> object_ids;
    //object_ids.push_back(collision_object.id);
    //planning_scene_interface.removeCollisionObjects(object_ids);
    visual_tools.deleteAllMarkers();

    ros::shutdown();
    return 0;
}
