The box grasping workspace for master thesis.

Content:
- [dependecies](#dependecies)

#### help command

```python
'''
realsense
'''
## driver
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true

rosrun pcl_tutorial save_depth_with_mask /camera/color/image_raw /camera/depth/image_rect_raw imgs


# main
rosrun pcl_tutorial main reso/box_small/thin.pcd -o 0.7 -d 0.01 -t 1000 -n 100 -it 100

# laser line
roslaunch laser_line_extraction example.launch
rosrun pcl_tutorial counter_pose 2.0 0.65,0.75
# tf
rosrun tf static_transform_publisher  0.00662214  0.0339903 -0.0687947  0.919409 -0.392716 0.0021969 0.0213767 arm_left_link_7_t camera_rgb_optical_frame 100

hokuyo: 40hz

'''
sda
'''
## driver
roslaunch motoman_sda5f_support robot_interface_streaming_sda5f.launch 
roslaunch motoman_sda5f_moveit_config demo_real.launch 

```

initial states:

    [arm_left_joint_1_s, arm_left_joint_2_l, arm_left_joint_3_e, arm_left_joint_4_u, arm_left_joint_5_r,arm_left_joint_6_b, arm_left_joint_7_t]
    position: [2.6937308311462402, -1.7499834299087524, -1.922503113746643, 0.04039989039301872, 0.8331186175346375, -1.1421606540679932, 1.8852884769439697]

#### ip

sda: 192.168.1.10
hokuyo: 192.168.1.11
pc-sda: 192.168.1.87

#### tf

The sensor link is published by  `static_transform_publisher` in `motoman_sda5f_moveit_config/launch/motoman_sda5f_moveit_sensor_manager.launch`

sensor list:
    - asus camera: 0.00662214  0.0339903 -0.0687947  0.919409 -0.392716 0.0021969 0.0213767 /arm_left_link_7_t /camera_rgb_optical_frame 100
    - hokuyo laser: 0.43 -0.15 0.68 0 0 0 1 /world /laser 100

#### model size

right gripper: 
- gripper legth: 210mm
- gripper bottom to end-effector: 134.7mm
- gripper finger width: 5mm 

#### camera calibration

```
# asus 2
*** Added sample 152, p_x = 0.661, p_y = 0.708, p_size = 0.253, skew = 0.724
('D = ', [-0.010767126205035457, 0.02878932007658363, -0.007516792631713471, -0.003062193530694651, 0.0])
('K = ', [536.3783314976719, 0.0, 325.59688771858265, 0.0, 539.4028276436596, 240.028337450314, 0.0, 0.0, 1.0])
('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
('P = ', [541.3510131835938, 0.0, 323.52929686348944, 0.0, 0.0, 542.7696533203125, 236.24413879678468, 0.0, 0.0, 0.0, 1.0, 0.0])
```

## dependecies

#### ros package inside

- [ork_ws]()
- motoman sda5f driver
- [calibration](https://github.com/lixiny/Handeye-Calibration-ROS)
- [laser_line_extration](https://github.com/kam3k/laser_line_extraction)

#### ros package extral

- [yeebot_core](https://github.com/YeeKal/yeebot)
- openni2 driver
- realsense driver
- hokuyo driver: urg-node
- [laser_geometry](https://wiki.ros.org/laser_geometry)

#### system package

- [PCL]() along with ros
- [OpenCV]() along with ros
- [Sophus](https://github.com/stonier/sophus) for `calibration`
- [Ceres](http://ceres-solver.org/) for `calibration`
- [gflags](https://github.com/gflags/gflags) for `calibration`


