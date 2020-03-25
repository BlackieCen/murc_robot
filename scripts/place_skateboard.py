#!/usr/bin/env python
# --------------------------------------
# file:      place_skateboard.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     place the skateboard near the first end of profile
# --------------------------------------------------
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseActionResult
from grasp_object import *
import tf
from RPY2RotationVector import rpy2rv
from std_msgs.msg import String
from ur_msgs.msg import RobotModeDataMsg
from murc_robot.msg import RobotMoveAction, RobotMoveGoal, RobotMoveResult
import actionlib
from rg2_gripper_switcher import GripperController


class PlaceSkateboard:
    def __init__(self):
        self.sample_count = 30  # 1s*30FPS
        self.counter = 0
        self.sample_succeed = False
        self.x_array = []
        self.y_array = []
        self.z_array = []
        self.qx_array = []
        self.qy_array = []
        self.qz_array = []
        self.qw_array = []
        self.goal_sub = rospy.Subscriber("place4skateboard", Pose, self.place4skateboard_callback)
        self.place_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size = 1)
        self.ps_only_move_once = 0
        self.pose_finished = False
        self.place_client = actionlib.SimpleActionClient('robot_move', RobotMoveAction)
        self.place4skateboard_str = String()
        self.change_protection = False
        self.gc = GripperController()

    def place4skateboard_callback(self, pose2go):
        position = pose2go.position
        orientation = pose2go.orientation
        # goal to approach for MiR:
        # position: x = x0, y = y0, z = 0.0
        # orientation(quaternion): x=0.0 y=0.0 z=z1 w=w1
        while self.ps_only_move_once == 1:
            if self.counter < self.sample_count:
                self.sample(position, orientation)
                self.counter += 1
            else:
                self.sample_succeed = True

            if self.sample_succeed:
                m_x, std_x = statistic_analysis(self.x_array)
                m_y, std_y = statistic_analysis(self.y_array)
                m_z, std_z = statistic_analysis(self.z_array)
                m_qx, std_qx = statistic_analysis(self.qx_array)
                m_qy, std_qy = statistic_analysis(self.qy_array)
                m_qz, std_qz = statistic_analysis(self.qz_array)
                m_qw, std_qw = statistic_analysis(self.qw_array)

                if std_x < 15 and std_y < 15:
                    print("  Object locked...")
                    place_position = np.array([m_x[0], m_y[0], m_z[0]])
                    place_orientation = [m_qx[0],m_qy[0],m_qz[0],m_qw[0]]
                    offset_position = np.array([-0.3844, +0.2899-0.04, +0.11-0.010]) #60cm results from the relative displacement of gripper and profile
                    place_position = place_position + offset_position # z=-326
                    # print(place_position)
                    offset_theta_z = +6.0/180.0*math.pi
                    ps_quaternion = [m_qx[0],m_qy[0],m_qz[0],m_qw[0]]
                    euler = tf.transformations.euler_from_quaternion(ps_quaternion)
                    roll = euler[0]
                    pitch = euler[1]
                    yaw = euler[2] + offset_theta_z
                    (rx, ry, rz) = rpy2rv(roll, pitch, yaw)
                    rv = [rx, ry, rz]
                    # place4skateboard_pose = [m_x[0], m_y[0], m_z[0]+0.2, rx, ry, rz]
                    place4skateboard_pose = list(place_position)+rv
                    
                    if not self.change_protection:
                        # payload_str = "set_payload_mass(3.75)\n"
                        self.place4skateboard_str = "movel(p" + str(place4skateboard_pose) + ", a=0.2, v=0.2, t=0, r=0)"
                        self.change_protection = True
                    
                    rospy.sleep(10.0)
                    gripper_control_finished = self.gc.close_gripper()
                    while True:
                        rospy.sleep(0.1)
                        gripper_control_finished = self.gc.gripper_switch_finished
                        if gripper_control_finished:
                            break
                    # place4skateboard_str = pose2_grasping_preparation ='movej([1.57, -1.0472, -2.0944, -1.3963, 1.745, 3.1416], a=0.2, v=1, t=0, r=0)'
                    rospy.sleep(5.0)
                    self.place_client.wait_for_server()
                    goal = RobotMoveGoal()
                    goal.move_command = self.place4skateboard_str
                    self.place_client.send_goal(goal)
                    self.place_client.wait_for_result()
                    result = self.place_client.get_result()
                    print(result)
                    while not self.pose_finished:
                        rospy.sleep(0.1)
                        msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
                        # print(msg.is_program_running)
                        self.pose_finished= not msg.is_program_running
                    self.ps_only_move_once = 0
                    gripper_control_finished = self.gc.open_gripper()
                    while True:
                        rospy.sleep(0.1)
                        gripper_control_finished = self.gc.gripper_switch_finished
                        if gripper_control_finished:
                            break
                
                self.sampled_data_clean()


    def sample(self, point, quaternion):
        self.x_array = np.append(self.x_array, point.x)
        self.y_array = np.append(self.y_array, point.y)
        self.z_array = np.append(self.z_array, point.z)
        self.qx_array = np.append(self.qx_array, quaternion.x)
        self.qy_array = np.append(self.qy_array, quaternion.y)
        self.qz_array = np.append(self.qz_array, quaternion.z)
        self.qw_array = np.append(self.qw_array, quaternion.w)

    def sampled_data_clean(self):
        self.x_array = []
        self.y_array = []
        self.z_array = []
        self.qx_array = []
        self.qy_array = []
        self.qz_array = []
        self.qw_array = []
        self.counter = 0
        self.pose_finished = 0


if __name__ == '__main__':
    rospy.init_node("place_skateboard")
    PS = PlaceSkateboard()
    PS.ps_only_move_once = 1
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop approaching to the goal object!")
