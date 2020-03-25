# murc_robot

[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

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

## Background

This strategy was developed in my master thesis *Development of an Image-Based Gripping Strategy for Cooperative Object Transport using Mobile Robots* in MATCH LUH. As a preliminary project of cooperative object transport using several mobile robots, this thesis focuses on the transport of an aluminium profile by the cooperation of one mobile robot and a passive flatbed trolley. The proposed strategy is able to help the robot autonomously accomplish the transport task of a object whose weight is beyond the maximum payload of robot arm.

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
3. run object detection program in PC:  
   ```sh
   roslaunch murc_robot object_detector.py
   ```
4. visualization of camera view und the result of objetdetection in Laptop:  
   ```sh
   rosrun murc_robot img_displayer.py
   ```
5. monitor current gripping process in state machine in Laptop:  
   ```sh
   rosrun smach_viewer smach_viewer.py
   ```
6. launch state machine:  
   ```sh
   rosrun murc_robot imagebased_grasping_smach4.py
   ```

## Package Overview

Package | Description
------------- | -------------
mir_driver    |   The mir driver provides most of the necessary topics and functionalities for either Odometry or SLAM navigation, including the move_base action server, which can be used to execute goal requests. Since velocities of the wheels are not published directly, a seperate node is launched, in order to provide those.
ur_driver     | Old ros driver providing urdf-files and core functionalities. Does not have to be started
ur_modern_driver | Updated version of ur_driver. Needed for communication with robot controller using ur_srcipt messages. roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.12.90. Arm can be used in different modes: movel, movej, speedl, speedj. Translations currently have to be implemented by yourself. Specific commands can be seen below: XXXXX. Note: The robot hardware controller can only handle low command frequency around under 100Hz. which, so far, is sufficient.
ur5moveit_config | Only needed for differential kinematics. Especially jacobian. Might throw some errors when loaded before ur_modern_driver.
RG2_gripper_server | To use the RG2 Gripper, the action server must be started. Can be called through a client, which requires an opening width. It's implemented using the python urx library, which instantiates an individual robot object. Therefore the python-urx folder must be located in the mur_robot src folder
mur_params | Important parameters can be launched using the params.launch file. It currently stores arm poses as well as navigaiton poses. Mind the namespacing
teleop_keyboard | Native node to move the mobile platform using twist velocity commands which are published to the /cmd_vel topic.
teleop_robot | Modified version of teleop_twist_keyboard, but can be used to publish/ load cartesian target poses. 
nullspace_controller5.py | Controller to maintain the current cartesian position of the tcp. How it works: tcp position is transformed in to world frame. World frame can either be Odometry or AMCL-based. Controller always moves arm to specified world posisiton. (needs params.launch)
move_base_trajectory | Script file to define an odometry based path for the mobile platform. Sends /cmd_vel commands for a specified direction (x_lin and z_angular) for a desired time. 
LoadObject | Action_server to Load an Object. Performs arm motions and gives feedback during runtime. Current status: not ready yet.

