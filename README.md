The box grasping workspace for master thesis.

```python
'''
realsense
'''
## driver
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud align_depth:=true

rosrun pcl_tutorial save_depth_with_mask /camera/color/image_raw /camera/depth/image_rect_raw imgs


# main
rosrun pcl_tutorial main reso/box_small/thin.pcd -o 0.7 -d 0.01 -t 1000 -n 100 -it 100

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

#### camera calibration

```
# asus

('D = ', [0.04869817149535155, -0.13796301358544696, 0.0074649186063052595, 0.003718692021264184, 0.0])
('K = ', [537.3745458137307, 0.0, 325.28760172950103, 0.0, 534.8118748679537, 249.0282301349994, 0.0, 0.0, 1.0])
('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
('P = ', [537.691162109375, 0.0, 327.7376611684158, 0.0, 0.0, 537.2880859375, 251.69179250104935, 0.0, 0.0, 0.0, 1.0, 0.0])
```
