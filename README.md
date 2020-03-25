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
	- [Generator](#generator)
- [Badge](#badge)
- [Example Readmes](#example-readmes)
- [Related Efforts](#related-efforts)
- [Maintainers](#maintainers)
- [Contributing](#contributing)
- [License](#license)

## Background

This strategy was developed in my master thesis *Development of an Image-Based Gripping Strategy for Cooperative Object Transport using Mobile Robots*. As a preliminary project of cooperative object transport using several mobile robots, this thesis focuses on the transport of an aluminium profile by the cooperation of one mobile robot and a passive flatbed trolley. The proposed strategy is able to help the robot autonomously accomplish the transport task of a object whose weight is beyond the maximum payload of robot arm.

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
2. adjust tcp-coordinate system:   (installation-- RG configuration-- rotate the gripper twice in GUI, then RZ becomes 0. )
3. load placing program:   program-- load program-- open */programms/Cen/Placing.urp*

### What is this Robot:

The MuR205 Robot is composed of a camera, a robot arm, a gripper, a mobile platform and a computer. It combines the capabilities of a depth camera (Intel RealSense Depth Camera D435), a 6-axis lightweight robot arm(UR5) and a gripper (OnRobot RG2) as well as a mobile platform (MiR200). 

### How is it set up
The robot consists of the UR5 and the MiR200. The UR5 Controller was fully integrated to the mobile platform. The Software is implemented in ROS, while using Python.
Each System, arm and mobile base, are using their own ROS drivers. An on-board computer and ROS Network acts as ROS-Master and connects both systems. Remote control can be done via SSH-connection from an external device. 


### How to use the robot
Log into MATCHWLAN. Currently use 
```bash
ssh mur
match2019
```

pro Tip: use Terminator and its capabilities: alt+a lets you write in all terminals simultaneous, alt+o reverses. alt+(up/down/left/right) lets you switch between terminals.
Open 

```bash
$ sudo gedit ~/.bashrc
```

and uncomment 'ForcePromptColor' line to have a coloured prompt.

To control the robot, A LOT of nodes have to be started in different terminals. This makes the System very adaptable to changes and very easy for development/debugging.

All bash-commands from now on are done remotely on the on-board computer via ssh:
#### 1 ROS-Core/Master

```bash
$ roscore
```

#### 2: Start Mir Driver


```bash
$ roslaunch mir_driver mir.launch
```
How to use the driver:

Use the /cmd_vel topic to publish velocity commands for mobile platform with the twist message that defines a linear x and angular z velocity.
Use the movebase action server to send goals within the map. An example is provided with moveBase_client.py It shows how to sent action requests to the action server. Note that the desired pose has to be loaded into the parameter server. Therefore load param.launch (see package overview, mur_params)

pose_subscriber.py gives an overview on how to save a pose. Do not trust any of the given poses, since they may not be up to date anymore, relatively to the current real environment.

base_trajectory commander shows a very simple approach on how to define trajectories without the action server, instead using the cmd_vel topic. Note the destination of the commands. I have different options implemented on how to control motions. Those are the 3 following
1. regular cmd_vel topics, for example with rosrun teleop_robot teleop_keyboard
2. whenever I used the differential control mode, i published my command as a parameter(which is not ideal), which was then read by the controller. This means, a differential velocity controller has to be running simultaneously, this could be for example feedforward_Nullspace_Controller.py . with rosrun mur_robot teleop_robot_mir.py , you can command individual velocity commands to the controller
3. I encoutered, that the jerk of the mobile platform was very high and could not be changed easily. I found the yocs_velocity_smoother package, which smoothes veloctity commands, in order reduce the jerk. Therefore, I redirected the topics using cmd_remapper.py from cmd_vel to cmd_vel_smooth, so that every command was smoothed before. Unfortunately, this package does not work properly.

#### 3: Start UR5 Driver [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)

```bash
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.12.90
```
This driver is very rudimental, but still gives full control for the ur robot. The driver sends ur-script commands to the controller, as you can see in ur_test.py. ur_script is the language used when programming via the touchpad. You can not get direct feedback if a motion is successfully executed. Therefore, you have check the robot_mode_state topic, which gives at least a little insight of what the robot is doing. Therefore, the while loop is added after a ur command, in order to wait, until the robot is done moving.
velocity_controller.py also shows an example on how to use the velocity commands speedl/speedj in a while loop (USE WITH CAUTION!!)
Saving a pose is also explained in pose_subscriber.
Whenever a diferential velocity controller is active, teleop_robot_cartesian_position.py can be used to command cartesian velocities for the arm relatively to the world frame coordinate system.

guide_robot.py shows the use of the wrench topic, which publishes forces and torques applied to the robots body. 


Info:
This Driver requires prior installation of the [old driver](https://github.com/ros-industrial/universal_robot), since it uses its core components. 
Update Info:
Recently, there has been a lot of progress in the driver development: The ur_modern_driver is now outdated and an [official ROS-Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) from Universal Robots was released.
If you're interested, please check out if the current controller polyscope version is compatible with the robot in use at match. It also requires to patch a real time kernel, btw.
The new driver features a lot more functionalities, than the driver used in this project. Nevertheless, the ur_modern_driver was the basis for the official driver.
#### 4: Start MoveIt! Configuration

```bash
$ roslaunch ur5_moveit_config move_group.launch
```
This launch file is needed when you use differential kinematics, since this package provides the jacobian matrix for the ur5
Always start after ur_modern_driver!

## The System is now ready to take commands from you

Try
```bash
roslaunch mur_robot start.launch
```

if you like to save time ;)

### What are its capabilities

The full capabilities can be seen in [this video](https://www.youtube.com/watch?v=v1bzNrgP8kg), where feedForward_Nullspace_Controller_PID(degbug).py is used

#### How the Controller is implemented:

The Controller loop is implemented in a subriber callback thread. It tries to keep the target position of the end effector relatively to the world frame.

Therefore, all the needed topics are subscribed with a message_filter.

1. The jacobian of the current state of the robot is computed. The mathematical details can be looked up in my Masters Thesis.

2. Then the redundancies are solved with the extended jacobian method

3. target_position for the End effector is updated

4. the resulting target velocity is calculated from the change in the target position.

5. computeJointSpeeds is used to compute the inverse of the extended jacobian (feedforward part)

6. positionController is just used to have a closed loop, since the feedforward loop is not closed and therefore leads to drift over time.

7. actuateJoints sends out the actual velocity commands to both of the sub systems



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


