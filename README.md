The box grasping workspace for master thesis.

Content:
- [dependecies](#dependecies)
- [hardware](#hardware)

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
roslaunch pcl_tutorial filter_laser.launch

roslaunch laserline laserline.launch 

roslaunch laser_line_extraction example.launch
rosrun pcl_tutorial counter_pose 2.0 0.65,0.75
# tf
rosrun tf static_transform_publisher  0.00662214  0.0339903 -0.0687947  0.919409 -0.392716 0.0021969 0.0213767 arm_left_link_7_t camera_rgb_optical_frame 100

## sda driver
roslaunch motoman_sda5f_support robot_interface_streaming_sda5f.launch 
roslaunch motoman_sda5f_moveit_config demo_real.launch 

```

#### technique

1. laser detection
    - laser points differ
    - low-pass filter
    - line detection
2. camera recognition
    - hsv segmentation
    - 6D pose estimation: ICP+ Super4PCS
3. motion planning



## hardware

#### sda

initial states:

    [arm_left_joint_1_s, arm_left_joint_2_l, arm_left_joint_3_e, arm_left_joint_4_u, arm_left_joint_5_r,arm_left_joint_6_b, arm_left_joint_7_t]
    position: [2.6937308311462402, -1.7499834299087524, -1.922503113746643, 0.04039989039301872, 0.8331186175346375, -1.1421606540679932, 1.8852884769439697]

initial_states(2019-12-05):

    [arm_left_joint_1_s, arm_left_joint_2_l, arm_left_joint_3_e, arm_left_joint_4_u, arm_left_joint_5_r,
  arm_left_joint_6_b, arm_left_joint_7_t]
position: [-0.022554073482751846, 1.3352922201156616, 1.1472201347351074, 0.19595465064048767, 0.7072410583496094, -0.9488791227340698, 1.834251046180725]

#### laser

hokuyo: 40hz

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
- [laser_line_segment]()

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
- [ncurses](): sudo apt-get install libncurses5-dev

## pose estimation paras

```
items           time/ms   score/e-5
super4pcs+icp   479         3.264
                498         3.971
super4pcs       289         3.682
                241         3.98
                281         4.337
                688         3.45
super4pcs(before)212        3.9
                428         3.25
                374         4.62
super4pcs+icp   1460        2.73
                1380        2.82
                1040        2.75
ppf             400         k
                1020
                876
```


