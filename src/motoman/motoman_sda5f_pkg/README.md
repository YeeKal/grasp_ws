# Instruction for SDA5F in Lab224

#### ip

#### model

为了避免每次更换抓手就要更改配置文件，现统一如下：
- 模型描述文件为`sda5f_with_gripper`
- 抓手连杆名称统一为`arm_[right/left]_link_gripper`
- 抓手模型放在`meshes/gripper`
- 头部连杆同意命名为`head_link`
- 碰撞矩阵在`motoman_sda5f.srdf`


## motoman_sda5f_pkg from QQfly
This is a ROS package for motoman SDA5F dual-arm robot. And this package is only validated on Ubuntu 14.04 STL + ROS indigo
<div  align="center">
    <img src="/motoman_sda5f_support/pic/SDA5F.jpg" width = "300" height = "300" alt="SDA5F" />
</div>
<br>

#### motoman_sda5f_support
This package contains configuration data, 3D models and launch files for Motoman SDA5F manipulators.

#### motoman_sda5f_moveit_config
An automatically generated package with all the configuration and launch files for using the motoman_sda5f with the MoveIt Motion Planning Framework. industrial_robot_simulator is launched as an fake controller.

#### Install

```
1. cd ~/catkin_ws/src
2. git clone git@github.com:qqfly/motoman_sda5f_pkg.git
3. rosdep install --from-paths . --ignore-src --rosdistro indigo
4. catkin_make
```

#### In Simulation
1.Display SDA5 in Rviz: <br>
```
roslaunch motoman_sda5f_support test_sda5f.launch
```

2.Use MoveIt with simulated robot:<br>
```
roslaunch motoman_sda5f_moveit_config demo_fake.launch
or
roslaunch motoman_sda5f_moveit_config demo_sim.launch 
```
<div  align="center">
    <img src="/motoman_sda5f_support/pic/rviz.png" width = "400" height = "300" alt="sim_SDA5F" />
</div>

#### Real Robot
1.Download motoman_driver for dual-arm FS100 contrller.
```
git clone git@github.com:qqfly/motoman.git -b dual_arm_fs100_indigo
```
2.Follow the instructions from [ROS](http://wiki.ros.org/motoman_driver/Tutorials/indigo/InstallServer)<br>
3.Set IP for your computer
<div  align="center">
    <img src="/motoman_sda5f_support/pic/IP_for_SDA.png" width = "300" height = "300" alt="sim_SDA5F" />
</div>
4.Connect robot with an Ethernet cable. <br>
5.Launch motoman_driver
```
roslaunch motoman_sda5f_support robot_interface_streaming_sda5f.launch
```
6.Launch MoveIt
```
roslaunch motoman_sda5f_moveit_config demo_real.launch
```




