//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <time.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <obj_srv/obj_6d.h>
#include <motoman_msgs/WriteSingleIO.h>

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

using namespace std;



bool MPlanning(string planning_group, string reframe, geometry_msgs::Pose target_pose)
{
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    move_group.setPoseReferenceFrame(reframe);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPoseTarget(target_pose);
    move_group.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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
bool CPlanning(string planning_group, string reframe, vector<geometry_msgs::Pose> waypoints)
{
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    move_group.setPoseReferenceFrame(reframe);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setStartStateToCurrentState();
    //ROS_INFO("Reference end frame: %s", move_group.getEndEffectorLink().c_str());
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; //0.0
    const double eef_step = 0.005;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan C (cartesian path) (%.2f%% acheived)", fraction * 100.0);
    if (fraction < 1.0)
    {
        return 0;
    }
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
    //ROS_INFO("Visualizing plan C (again)");
    move_group.execute(my_plan);
    return 1;
}
bool JPlanning(string planning_group, string reframe, vector<double> joint_group_positions)
{
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit_msgs::DisplayTrajectory display_trajectory;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setPoseReferenceFrame(reframe);
    move_group.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan Joint (pose goal) %s", success ? "" : "FAILED");
    if (success)
    {
        ROS_INFO("Visualizing plan Joint (again)");
        //display_trajectory.trajectory_start = my_plan.start_state_;
        //display_trajectory.trajectory.push_back(my_plan.trajectory_);
        //display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        //sleep(1.0);
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
    ros::AsyncSpinner spinner(4);
    spinner.start();
    static const std::string PLANNING_GROUP = "arm_right";
    moveit::planning_interface::MoveGroupInterface arm_right(PLANNING_GROUP);
    ros::Publisher pub_co = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";
    co.id = "desk";
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);
    ros::WallDuration(1.0).sleep();
    ROS_INFO("Add desk!");
    co.id = "desk";
    co.operation = moveit_msgs::CollisionObject::ADD;
    geometry_msgs::Pose co_pose;
    co_pose.position.x = 0.6;
    co_pose.position.y = -0.15;
    co_pose.position.z = 0.0;
    co_pose.orientation.x = 0.0;
    co_pose.orientation.y = 0.0;
    co_pose.orientation.z = 0.0; //sin((45.0 / 2) * 3.1415 / 180)
    co_pose.orientation.w = 1.0; //cos((45.0 / 2) * 3.1415 / 180)
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.65;
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(co_pose);
    co_pose.position.y = -0.75;
    co_pose.position.z = 0.4;
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 1.0;
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(co_pose);
    pub_co.publish(co);

    ros::ServiceClient client_obj = node_handle.serviceClient<obj_srv::obj_6d>("/obj_poses");
    obj_srv::obj_6d srv_poses;
    srv_poses.request.start = 1;
    ros::ServiceClient client_griper = node_handle.serviceClient<motoman_msgs::WriteSingleIO>("/write_single_io");
    motoman_msgs::WriteSingleIO srv_io_open;
    motoman_msgs::WriteSingleIO srv_io_close;
    srv_io_open.request.address = 10170;
    srv_io_open.request.value = 0;
    srv_io_close.request.address = 10170;
    srv_io_close.request.value = 1;

    tf::TransformListener lr;
    tf::TransformBroadcaster br;
    tf::StampedTransform transform_camera;
    geometry_msgs::Pose place_pose;
    // place_pose.orientation.x = 0.61615;
    // place_pose.orientation.y = 0.67125;
    // place_pose.orientation.z = -0.29564;
    // place_pose.orientation.w = 0.28702;
    // place_pose.position.x = 0.55994;
    // place_pose.position.y = -0.54371;
    // place_pose.position.z = 0.47776 - 0.14;
    // place_pose.orientation.x = 0.72012;
    // place_pose.orientation.y = -0.69358;
    // place_pose.orientation.z = -0.010098;
    // place_pose.orientation.w = -0.016568;
    // place_pose.position.x = 0.58629;
    // place_pose.position.y = 0; //-0.24286;
    // place_pose.position.z = 0.57046 - 0.14;
    /*vector<double> right_home = {0.6872689127922058, 0.8881444931030273, 0.8256005644798279,
                                 -0.6860842704772949, -1.140127420425415, -1.044623613357544, 0.44}; //-2.749959707260132*/
    vector<double> place_joint_1 = {1.6275991201400757, 1.0104528665542603, 1.1142774820327759,
                                     0.09823551774024963, -2.0295932292938232, -1.193350911140442, 1.9943830966949463};
    vector<double> place_joint_2 = {1.5616682767868042, 0.9471343755722046, 1.168559193611145, 
                                    -0.08802922815084457, -2.10147762298584, -1.2146024703979492, 2.1061296463012695};
    vector<double> place_joint_3 = {1.5568993091583252, 0.9196897745132446, 1.2068631649017334,
                                     -0.20455101132392883, -2.167727470397949, -1.258863925933838, 2.21694016456604};
    vector<double> place_joint_4 = {1.5297737121582031, 0.8892379999160767, 1.261387825012207, 
                                    -0.3189616799354553, -2.2251226902008057, -1.2833731174468994, 2.307133197784424};
    vector<double> place_joint_5 = {1.539615511894226, 0.8638134002685547, 1.2895917892456055,
                                     -0.3988957405090332, -2.2837023735046387, -1.3167587518692017, 2.385418176651001};
    vector<vector<double>> place_joints(5);
    place_joints[0] = place_joint_1; 
    place_joints[1] = place_joint_2;
    place_joints[2] = place_joint_3;
    place_joints[3] = place_joint_4;
    place_joints[4] = place_joint_5;
    vector<double> right_home = {0.8583457469940186, 1.1640939712524414, 0.5965666174888611,
                                 -0.4981032907962799, -1.085329294204712, -1.0616180896759033, 0.5538710951805115};
    bool success0 = JPlanning(PLANNING_GROUP, "base_link", right_home);
    if (!success0)
    {
        return 0;
    }
    int item_num = 5;
    for (int item = pick_id - 1; item < item_num; item++)
    {

        ROS_INFO("Pick Ready !");
        client_griper.call(srv_io_open);

        //10170 ---- 30030
        // tf::Transform transform_gripper;
        // transform_gripper.setOrigin(tf::Vector3(0.0, 0.0, 0.1));
        // //transform_gripper.setRotation( tf::Quaternion(0.0, 0.0, -0.707, 0.707) );
        // tf::Matrix3x3 matrix_gripper = tf::Matrix3x3(0.0, 1.0, 0.0,
        //                                              0.0, 0.0, 0.0,
        //                                              1.0, 0.0, -1.0);
        // tf::Quaternion matrix_gripper_q;
        // matrix_gripper.getRotation(matrix_gripper_q);
        // transform_gripper.setRotation(matrix_gripper_q);

        ros::WallDuration(1.0).sleep();
        lr.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
        lr.lookupTransform("/base_link", "/camera_link", ros::Time(0), transform_camera);
        client_obj.call(srv_poses);
        geometry_msgs::PoseArray pose_array = srv_poses.response.obj_array;
        while (pose_array.poses.size() < 1)
        {
            client_obj.call(srv_poses);
            pose_array = srv_poses.response.obj_array;
        }
        geometry_msgs::Pose pose = pose_array.poses[0];
        tf::Transform transform_pose;
        transform_pose.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        transform_pose.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
        transform_pose = transform_camera * transform_pose;
        //cout<<"transform_pose: "<<transform_pose<<endl;
        tf::Transform transform_pick;
        tf::Transform transform_b;
        transform_b.setOrigin(tf::Vector3(0.023, 0.0, -0.043));
        transform_b.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf::Matrix3x3 matrix_object = tf::Matrix3x3(transform_pose.getRotation());
        tf::Vector3 matrix_object_z = matrix_object.getColumn(2);
        tf::Vector3 gripper_x = matrix_object_z;
        tf::Vector3 gripper_y = matrix_object_z.cross(tf::Vector3(0, 0, 1));
        tf::Vector3 gripper_z = gripper_x.cross(gripper_y);
        tf::Matrix3x3 matrix_gripper = tf::Matrix3x3(gripper_x.getX(), gripper_y.getX(), gripper_z.getX(),
                                                     gripper_x.getY(), gripper_y.getY(), gripper_z.getY(),
                                                     gripper_x.getZ(), gripper_y.getZ(), gripper_z.getZ());
        tf::Quaternion matrix_gripper_q;
        matrix_gripper.getRotation(matrix_gripper_q);
        transform_pick.setRotation(matrix_gripper_q);
        transform_pick.setOrigin(transform_pose.getOrigin());
        transform_pick = transform_pick * transform_b;
        ROS_INFO("Object Pose Estimation Completed !");
        geometry_msgs::Pose pick_pose;
        pick_pose.orientation.x = transform_pick.getRotation().getX();
        pick_pose.orientation.y = transform_pick.getRotation().getY();
        pick_pose.orientation.z = transform_pick.getRotation().getZ();
        pick_pose.orientation.w = transform_pick.getRotation().getW();
        pick_pose.position.x = transform_pick.getOrigin().getX();
        pick_pose.position.y = transform_pick.getOrigin().getY();
        pick_pose.position.z = transform_pick.getOrigin().getZ();
        ROS_INFO_STREAM("pick_pose is " << pick_pose);
        bool success1 = CPlanning(PLANNING_GROUP, "base_link", {pick_pose}); //M
        if (!success1)
        {
            return 0;
        }
        vector<geometry_msgs::Pose> waypoints1;
        geometry_msgs::Pose pose_waypoints1;
        pose_waypoints1.position.z += 0.039;
        waypoints1.push_back(pose_waypoints1);
        bool success2 = CPlanning(PLANNING_GROUP, "arm_right_link_tool0", waypoints1);
        if (!success2)
        {
            return 0;
        }
        client_griper.call(srv_io_close);
        // ros::WallDuration(0.5).sleep();
        ROS_INFO("Pick it !");
        vector<geometry_msgs::Pose> waypoints2;
        geometry_msgs::Pose pose_waypoints2;
        pose_waypoints2.position.z -= 0.07;
        waypoints2.push_back(pose_waypoints2);
        bool success3 = CPlanning(PLANNING_GROUP, "arm_right_link_tool0", waypoints2);
        if (!success3)
        {
            return 0;
        }

        //bool success4 = MPlanning(PLANNING_GROUP, "base_link", place_pose);
        bool success4 = JPlanning(PLANNING_GROUP, "base_link", place_joints[item]);
        if (!success4)
        {
            return 0;
        }
        vector<geometry_msgs::Pose> waypoints3;
        geometry_msgs::Pose pose_waypoints3;
        pose_waypoints3.position.z += 0.045;
        waypoints3.push_back(pose_waypoints3);
        bool success5 = CPlanning(PLANNING_GROUP, "arm_right_link_tool0", waypoints3);
        if (!success5)
        {
            return 0;
        }
        client_griper.call(srv_io_open);
        // ros::WallDuration(0.5).sleep();
        ROS_INFO("Place it !");
        vector<geometry_msgs::Pose> waypoints4;
        geometry_msgs::Pose pose_waypoints4;
        pose_waypoints4.position.z -= 0.055;
        waypoints4.push_back(pose_waypoints4);
        bool success6 = CPlanning(PLANNING_GROUP, "arm_right_link_tool0", waypoints4);
        if (!success6)
        {
            return 0;
        }
        //ros::WallDuration(0.5).sleep();

        success0 = JPlanning(PLANNING_GROUP, "base_link", right_home);
        if (!success0)
        {
            return 0;
        }
    }

    //lr.waitForTransform("/base_link", "/ar_marker_0", ros::Time(0), ros::Duration(10.0));
    //lr.lookupTransform("/base_link", "/ar_marker_0", ros::Time(0), transform_a);

    // tf::Matrix3x3 matrix_object = tf::Matrix3x3(transform_a.getRotation());
    // tf::Vector3 gripper_z = -matrix_object.getColumn(2);
    // tf::Vector3 gripper_x = -matrix_object.getColumn(1);
    // tf::Vector3 gripper_y = -matrix_object.getColumn(0);
    // tf::Matrix3x3 matrix_gripper = tf::Matrix3x3(gripper_x.getX(), gripper_y.getX(), gripper_z.getX(),
    //                                              gripper_x.getY(), gripper_y.getY(), gripper_z.getY(),
    //                                              gripper_x.getZ(), gripper_y.getZ(), gripper_z.getZ());
    // tf::Quaternion matrix_gripper_q;
    // matrix_gripper.getRotation(matrix_gripper_q);

    // geometry_msgs::Pose pick_pose;
    // pick_pose.orientation.x = matrix_gripper_q.getX();
    // pick_pose.orientation.y = matrix_gripper_q.getY();
    // pick_pose.orientation.z = matrix_gripper_q.getZ();
    // pick_pose.orientation.w = matrix_gripper_q.getW();
    // pick_pose.position.x = transform_a.getOrigin().getX();
    // pick_pose.position.y = transform_a.getOrigin().getY();
    // pick_pose.position.z = transform_a.getOrigin().getZ() + 0.04;
    // bool success1 = MPlanning(PLANNING_GROUP, "base_link", pick_pose);
    // if (!success1)
    // {
    //     return 0;
    // }

    // sleep(1.0);
    // std_srvs::Trigger trigger;
    // client_obj.call(trigger);
    // sleep(2.0);
    // lr.waitForTransform("/base_link", "/obj_frame", ros::Time(0), ros::Duration(10.0));
    // lr.lookupTransform("/base_link", "/obj_frame", ros::Time(0), transform_a);

    // tf::Transform transform_pick;
    // tf::Transform transform_b;
    // transform_b.setOrigin(tf::Vector3(0.01, 0.0, -0.1));
    // transform_b.setRotation(tf::Quaternion(0, 0, 0, 1));
    // tf::Matrix3x3 matrix_object = tf::Matrix3x3(transform_a.getRotation());
    // tf::Vector3 matrix_object_z = matrix_object.getColumn(2);
    // tf::Vector3 gripper_x = matrix_object_z;
    // tf::Vector3 gripper_y = matrix_object_z.cross(tf::Vector3(0, 0, 1));
    // tf::Vector3 gripper_z = gripper_x.cross(gripper_y);
    // tf::Matrix3x3 matrix_gripper = tf::Matrix3x3(gripper_x.getX(), gripper_y.getX(), gripper_z.getX(),
    //                                              gripper_x.getY(), gripper_y.getY(), gripper_z.getY(),
    //                                              gripper_x.getZ(), gripper_y.getZ(), gripper_z.getZ());
    // tf::Quaternion matrix_gripper_q;
    // matrix_gripper.getRotation(matrix_gripper_q);
    // transform_pick.setRotation(matrix_gripper_q);
    // transform_pick.setOrigin(transform_a.getOrigin());
    // transform_pick = transform_pick * transform_b;

    // geometry_msgs::Pose pick_pose;
    // pick_pose.orientation.x = transform_pick.getRotation().getX();
    // pick_pose.orientation.y = transform_pick.getRotation().getY();
    // pick_pose.orientation.z = transform_pick.getRotation().getZ();
    // pick_pose.orientation.w = transform_pick.getRotation().getW();
    // pick_pose.position.x = transform_pick.getOrigin().getX();
    // pick_pose.position.y = transform_pick.getOrigin().getY();
    // pick_pose.position.z = transform_pick.getOrigin().getZ();
    // bool success1 = MPlanning(PLANNING_GROUP, "base_link", pick_pose);
    // if (!success1)
    // {
    //     return 0;
    // }
    //home:1.006807804107666, 0.908708930015564, 0.47164592146873474, -0.5410091876983643, -1.2322118282318115, -1.3011258840560913, -2.6368870735168457
    // while (1)
    // {
    //     geometry_msgs::Pose pose1;
    //     pose1.orientation.x = 0.7561;
    //     pose1.orientation.y = 0.64927;
    //     pose1.orientation.z = 0.081736;
    //     pose1.orientation.w = -0.00853;
    //     pose1.position.x = 0.49521;
    //     pose1.position.y = -0.28148;
    //     pose1.position.z = 0.53339;

    //     clock_t start = clock();
    //     bool success1 = MPlanning(PLANNING_GROUP, "base_link", pose1);
    //     clock_t finish = clock();
    //     printf("Use Time1:%f\n",(double)(finish-start)/CLOCKS_PER_SEC);
    //     if (!success1)
    //     {
    //         return 0;
    //     }
    //     vector<geometry_msgs::Pose> waypoints1;
    //     geometry_msgs::Pose pose2;
    //     pose2.position.z += 0.10;
    //     waypoints1.push_back(pose2);
    //     start = clock();
    //     bool success2 = CPlanning(PLANNING_GROUP, "arm_right_link_tool0", waypoints1);
    //     finish = clock();
    //     printf("Use Time2:%f\n",(double)(finish-start)/CLOCKS_PER_SEC);
    //     if (!success2)
    //     {
    //         return 0;
    //     }
    //     vector<geometry_msgs::Pose> waypoints2;
    //     geometry_msgs::Pose pose3;
    //     pose3.position.z -= 0.10;
    //     waypoints2.push_back(pose3);
    //     bool success3 = CPlanning(PLANNING_GROUP, "arm_right_link_tool0", waypoints2);
    //     if (!success3)
    //     {
    //         return 0;
    //     }
    //     geometry_msgs::Pose pose4;
    //     pose4.orientation.x = 0.75939;
    //     pose4.orientation.y = 0.64299;
    //     pose4.orientation.z = -0.036359;
    //     pose4.orientation.w = 0.092518;
    //     pose4.position.x = 0.71008;
    //     pose4.position.y = -0.14818;
    //     pose4.position.z = 0.76572;
    //     bool success4 = MPlanning(PLANNING_GROUP, "base_link", pose4);
    //     if (!success4)
    //     {
    //         return 0;
    //     }
    // }
    // bool successf = JPlanning(PLANNING_GROUP, "base_link", right_home);
    // if (!successf)
    // {
    //     return 0;
    // }

    // ros::Rate loop_rate(200);
    // while (ros::ok())
    // {
    //     br.sendTransform(tf::StampedTransform(transform_pick, ros::Time::now(), "base_link", "gripper_frame"));
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}
