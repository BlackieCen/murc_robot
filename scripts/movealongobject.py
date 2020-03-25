#!/usr/bin/env python
# --------------------------------------
# file:      movealongobject.py  compared to movealongobject_cmd.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     robot moves along the profile to its second end
#            the action is executed by the move_base (ActionServer)
#            Alternatively, the action can be executed by cmd_vel. It's robuster but only able to move forward/backward.
#            If necessary, just change the the line "from movealongobject import RelativeMovementMiR" in
#            imagebased_grasping_smach4.py to invoke another program.
# --------------------------------------------------
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseActionResult
from grasp_object import *
# from IO_test import *
from actionlib_msgs.msg import *
import tf


class RelativeMovementMiR:
    def __init__(self):
        self.displacement = [0., 0., 0.]
        self.current_pose = Pose()
        self.pose_sub = rospy.Subscriber("robot_pose", Pose, self.mir_pose_update)
        self.tl = tf.TransformListener()
        self.goal2alongobject = Pose()
        self.goal2alongobject_pub = rospy.Publisher("goal2alongprofile", Pose, queue_size=1)
        self.move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def mir_pose_update(self, pose_msg):
        self.current_pose = pose_msg

    def displacement_input(self):
        rospy.loginfo("|-----------------------------|")
        rospy.loginfo("|INPUT DISPLACEMENT FOR MiR TO GO:")
        # x, y, z = raw_input("Enter x,y,z[m]: ").split(",")
        self.displacement = list(map(float, raw_input("Enter a multiple value: ").split(",")))
        print("displacement is {}".format(self.displacement))
        # self.displacement = [x, y, z]

    def RMove_execute(self):
        displacement = self.displacement
        try:
            self.tl.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(5))  # original:5s
            (trans, rot_quat) = self.tl.lookupTransform('/map', '/base_link', rospy.Time(0))
        except Exception as e:
            rospy.logerr(
                'Failed during tf.TransformListener : ' + str(e) + ' \n\n Check if correct topics are being published!')
        map_T_baseMiR = tf.transformations.quaternion_matrix(rot_quat)
        map_T_baseMiR[0][3] = trans[0]
        map_T_baseMiR[1][3] = trans[1]
        map_T_baseMiR[2][3] = trans[2]

        displacemnet_vector_CS_baseMiR = np.array([[displacement[0]], [displacement[1]], [displacement[2]], [1]])
        # print (displacemnet_vector_CS_baseMiR)
        displacemnet_vector_CS_map = np.dot(map_T_baseMiR, displacemnet_vector_CS_baseMiR)
        self.goal2alongobject.position.x = displacemnet_vector_CS_map[0]  # + 0.0
        self.goal2alongobject.position.y = displacemnet_vector_CS_map[1]  # + 0.5
        self.goal2alongobject.position.z = displacemnet_vector_CS_map[2]  # ought to be 0.0

        self.goal2alongobject.orientation = self.current_pose.orientation
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = self.goal2alongobject.position
        goal.target_pose.pose.orientation = self.goal2alongobject.orientation
        print(goal.target_pose.pose)
        manual_confirmation = raw_input("Sure to move? (y/n): ")  # Python2
        if manual_confirmation == 'y':
            self.move_client.send_goal(goal)
            self.move_client.wait_for_result(rospy.Duration(60))
            ##################-- goal arrived --#################
            ## ------1-------##
            if (self.move_client.get_state() == GoalStatus.SUCCEEDED):
                rospy.loginfo("You have reached the destination")
                # self.MiR_only_move_once -= 1
                return True
            
            ## ------3-------##
            # self.move_client.wait_for_result()
            # result = MoveBaseResult()
            # while True:
            #     rospy.sleep(0.5)
            #     # result = self.move_client.get_result()
            #     result = rospy.wait_for_message("/move_base/result", MoveBaseActionResult)
            #     if result.status.text=="Goal reached.":
            #         rospy.loginfo("You have reached the destination")
            #         # self.MiR_only_move_once -= 1
            #         return True
        print ("Relative movement falied")
        return False


if __name__ == '__main__':
    rospy.init_node("move_along_profile")
    rmove = RelativeMovementMiR()
    rmove.displacement_input()
    rmove.RMove_execute()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop approaching to the goal object!")
