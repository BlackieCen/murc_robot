#!/usr/bin/env python
# --------------------------------------
# file:      approach2object.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     robot approaches the first end of the profile.
# --------------------------------------------------

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseActionResult
from grasp_object import *
from IO_test import *
from actionlib_msgs.msg import *


class Approach2Object:
    def __init__(self):
        self.sample_count = 90  # 3s*30FPS
        self.counter = 0
        self.sample_succeed = False
        self.x_array = []
        self.y_array = []
        self.z_array = []
        self.qx_array = []
        self.qy_array = []
        self.qz_array = []
        self.qw_array = []
        self.goal_sub = rospy.Subscriber("goal2approach", Pose, self.goal2approach_callback)
        self.move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_position = Point()
        self.goal_orientation = Quaternion()
        self.MiR_only_move_once = 0
        self.goal = MoveBaseGoal()

    def goal2approach_callback(self, pose2go):
        position = pose2go.position
        orientation = pose2go.orientation
        # goal to approach for MiR:
        # position: x = x0, y = y0, z = 0.0
        # orientation(quaternion): x=0.0 y=0.0 z=z1 w=w1
        while self.MiR_only_move_once ==1:
            if self.counter < self.sample_count:
                self.sample(position, orientation)
                self.counter += 1
                # print(self.counter)
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
                    self.goal_position.x = m_x[0]
                    self.goal_position.y = m_y[0]
                    self.goal_position.z = m_z[0]*0      # 0
                    self.goal_orientation.x = m_qx[0]*0  # 0
                    self.goal_orientation.y = m_qy[0]*0  # 0
                    self.goal_orientation.z = m_qz[0]
                    self.goal_orientation.w = m_qw[0]

                    #wait for the action server to come up
                    while(not self.move_client.wait_for_server(rospy.Duration.from_sec(5.0))):
                        rospy.loginfo("Waiting for the move_base action server to come up")

                    # self.move_client.wait_for_server()
                    # goal = MoveBaseGoal()
                    self.goal.target_pose.header.frame_id = "/map"
                    self.goal.target_pose.header.stamp = rospy.Time.now()
                    self.goal.target_pose.pose.position = self.goal_position
                    self.goal.target_pose.pose.orientation = self.goal_orientation

                    print(self.goal.target_pose.pose)
                    manual_confirmation = raw_input("Sure to move? (y/n): ")  # Python2
                    if manual_confirmation == 'y':
                        print(manual_confirmation)
                        self.move_client.send_goal(self.goal)
                        self.move_client.wait_for_result(rospy.Duration(60))
                        ##################-- goal arrived --#################
                        ## ------1-------##
                        if(self.move_client.get_state() ==  GoalStatus.SUCCEEDED):
                            rospy.loginfo("You have reached the destination")
                            self.MiR_only_move_once -= 1
                        ## ------2-------##
                        # while True:
                        #     rospy.sleep(0.5)
                        #     if(self.move_client.get_state() ==  GoalStatus.SUCCEEDED):
                        #         rospy.loginfo("You have reached the destination")
                        #         self.MiR_only_move_once -= 1
                        #         break
                        # rospy.sleep(5.0)
                        ## ------3-------##
                        # self.move_client.wait_for_result()
                        # # result = MoveBaseResult()
                        # check_counter = 0
                        # while True:
                        #     check_counter +=1
                        #     print(check_counter)
                        #     # rospy.sleep(0.5)s
                        #     # result = self.move_client.get_result()
                        #     result = rospy.wait_for_message("/move_base/result", MoveBaseActionResult)
                        #     print(result.status.text)
                        #     if result.status.text=="Goal reached.":
                        #         rospy.loginfo("You have reached the destination")
                        #         self.MiR_only_move_once -= 1
                        #         print(self.MiR_only_move_once)
                        #         break

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


if __name__ == '__main__':
    rospy.init_node("approach_to_object")
    a2o = Approach2Object()
    a2o.MiR_only_move_once = 1
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop approaching to the goal object!")
