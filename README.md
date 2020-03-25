# murc_robot

The *murc_robot* repo presents an image-based gripping strategy for cooperative object transport. 

## Table of Contents

- [Background](#background)
- [Introduction](#introduction)
    - [Task](#task)
    - [Robot](#robot)
    - [Framework](#framework)
- [Configurations in Hardware](#configurations-in-hardware)
- [Usage](#usage)
- [Package Overview](#package-overview)
- [Node Overview](#node-overview)

## Background

This strategy was developed in my master thesis *Development of an Image-Based Gripping Strategy for Cooperative Object Transport using Mobile Robots* in [match](https://www.match.uni-hannover.de/). As a preliminary project of cooperative object transport using several mobile robots, this thesis focuses on the transport of an aluminium profile by the cooperation of one mobile robot and a passive flatbed trolley. The proposed strategy is able to help the robot autonomously accomplish the transport task of a object whose weight is beyond the maximum payload of robot arm.

## Introduction
### Task
The strategy can guide a mobile robot through five steps in object transport:
1. detect the object to be transported
2. obtain its depth information
3. estimate 6D-pose of the object
4. plan a procedure to transport the object
5. execute the procedure.

### Robot
The MuR205 Robot is composed of a camera, a robot arm, a gripper, a mobile platform and a computer. It combines the capabilities of a depth camera (Intel RealSense Depth Camera D435), a 6-axis lightweight robot arm (UR5) and a gripper (OnRobot RG2) as well as a mobile platform (MiR200).

![robot and framework](https://github.com/BlackieCen/murc_robot/blob/master/pics/Topology.jpg)
### Framework
The communication between components of the robot is done via ROS. The programs ([/scripts](https://github.com/BlackieCen/murc_robot/tree/master/scripts)) are written in Python. In ROS， the components can share their own states with each other. As the following node diagram shows, the groups represent the camera, robot arm and platform respectively. The camera provides the images. The position of the profile results from the image processing. Then the components are allowed to detect the position of the profile and approach to, grip or deposit the profile accordingly. The operator can remotely run programs andcontrol the robot in a laptop, because PC in robot shares the ros-master with the laptop via SSH. The operator can also monitor the detected profile and the gripping process on a laptop.

![node diagram](https://github.com/BlackieCen/murc_robot/blob/master/pics/node_diagram.jpg)

## Configurations in Hardware
Before running programs, the hardware is supposed to be configured.

0. update of URCap from 1.0.1 to 1.10.1 (if an error "missing URCap" comes up, do this step):  
   robot setting-- URcaps-- "+" -- open *OnRobot/RG2-1.10.1_0/OnRobot-1.10.1.urcap*-- restart
1. load installation file (*murc.installation*) in teach pendant of UR5, then restart UR5
2. adjust tcp-coordinate system:  
   installation-- RG configuration-- rotate the gripper twice in GUI, then RZ becomes 0.
3. load placing program:  
   program-- load program-- open */programms/Cen/Placing.urp*

## Usage
1. run MiR driver in PC:  
   ```sh
   roslaunch mir_driver mir.launch
   ```
2. run MiR navigation (Rviz) in Laptop:  
   ```sh
   roslaunch mir_navigation mir_start.launch
   ```
3. run object detection program in PC, inclding camera driver and ur driver as well as serveral servers for object detection, transformation between different coordinate frames and components controlling:  
   ```sh
   roslaunch murc_robot object_detector.py
   ```
   if neccessary, run drivers separately
   ```sh
   roslaunch realsense2_camera rs_aligned_depth848x480.launch # camera driver
   roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.12.90 # ur driver
   ```
4. visualization of camera view und the result of objetdetection in Laptop:  
   ```sh
   rosrun murc_robot img_displayer.py
   ```
5. monitor current gripping process in state machine in Laptop:  
   ```sh
   rosrun smach_viewer smach_viewer.py
   ```
6. launch gripping state machine:  
   ```sh
   rosrun murc_robot imagebased_grasping_smach4.py
   ```

## Package Overview

Package | Description
------------- | -------------
mir_driver    |   The mir driver provides most of the necessary topics and functionalities for either Odometry or SLAM navigation, including the move_base action server, which can be used to execute goal requests. 
realsense2_camera     | This camera driver is able to stream all camera sensors and publish on the appropriate ROS topics.
ur_modern_driver | Updated version of ur_driver. Needed for communication with robot controller using *ur_srcipt* messages. Arm can be used in different modes: movel, movej, speedl, speedj. 

## Node Overview

Node | Description
------------- | -------------
approach2object | Robot approaches the first end of the profile.
functions_module | provides necessary functions to image processing, including transformation from uv-coordinates to 3D-coordinates in camera coordinate system
grasp_object | Robot arm grasps the object.
imagebased_grasping_smach4 | state machine to launch the whole gripping strategy
img_displayer | displays the result of object detection in the original image.
IO_test | can set&read IO ports' state. It is also able to start/stop the urp programs indirectly by setting or resetting port.
movealongobject | Robot moves along the profile to its second end. The action is executed by the move_base (ActionServer)
movealongobject_cmd | Robot moves along the profile to its second end. The action is executed by the by cmd_vel (ros topic)
obrobot_rg2_gripper | class of the rg2 gripper
place_skateboard | places the skateboard near the first end of profile
position_determination_server | service for calculating the position of object
position_publisher | invokes the service to calculate the position of object and publishes the results
profile_detection_v1_08 | algorithm of object detection and estimation of object' 6D-pose
rg2_gripper_switcher | can open/close the gripper
RG2Gripper_action_server | action server of the gripper
set_goal | calculates goals for mobile platform, robot arm and gripper
transformation_arm | take the transformation matrix camera_T_urtcp into transformation chain
transformation_handeye | published transformation matrix camera_T_urtcp --- result of hand eye calibration
ur_move_action_server | action server of robot arm
ur_pose_maker | change pose of the ur in joint space using movej
