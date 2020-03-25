#!/usr/bin/env python
# file:      grasp_object.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     robot arm grasps the object in 4 steps
#            1: x- und y-positioning  2:orientating
#            3: quick z-positioning (to the height z_o + 75mm)  z_o is the height of object
#            4: slow z-positioning (to the height z_o)
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float64, Int8, Bool
from ur_msgs.msg import RobotModeDataMsg
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import actionlib
from murc_robot.msg import RobotMoveAction, RobotMoveGoal, TcpPose, RobotMoveResult
from enum import Enum, unique
from sensor_msgs.msg import JointState
# libary for input timeout
import sys
import select
from StraightDownZ import *
import datetime


@unique
class GrippingProcedure(Enum):
	# all of following description is about whether the step is done or not
	nothing_done = 0
	xy_position_done = 1
	orientate_done = 2
	z_position_done = 3
	end = 4


class GraspObject:
	def __init__(self):
		# limits
		self.xy_offset = 100.0  # mm
		self.z_offset = 50.0  # mm
		self.z_finger = 25.0  # mm
		self.z_ground = -450  # mm
		self.z_min = -420.0  # mm # gripper open -450;gripper closed -420
		# 4 corners of base in ur_base coordinates system
		self.x_lowerlimit = -220.0
		self.x_upperlimit = +650.0
		self.y_lowerlimit = -415.0
		self.y_upperlimit = +140.0
		# flags
		self.safe_flag = False
		self.counter = 0
		self.x_array = []
		self.y_array = []
		self.z_array = []
		self.sample_count = 90  # 5s * 30FPS manually #3s*30FPS
		self.sample_counter = 0
		self.sample_succeed = False
		self.height_top_view = 0.5  # m
		self.is_robot_moving = False
		self.gripping_step = 0
		self.gamma = 0.0
		self.joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.tcp_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.target_offset = [0.0, 0.0, 0.0]
		self.number_of_samplings_to_skip= 0
		self.gripping_state_pub_counter = 90
		
		self.end_of_profile2grasp = False # 0:front end 	1:back end
		self.length_vecotr_CSbUR = [0.0,0.0,0.0]

		self.goal_sub = rospy.Subscriber("goal2catch", Point, self.callback)
		self.angle_sub = rospy.Subscriber('/angle_gamma', Float64, self.angle_update)
		self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_update)
		self.tcp_pose_sub = rospy.Subscriber('/tcp_pose', TcpPose, self.tcp_pose_update)
		self.move_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
		self.move_client = actionlib.SimpleActionClient('robot_move', RobotMoveAction)
		self.target_offset_sub = rospy.Subscriber('/target_offset', Point, self.target_offset_update)
		self.gripping_state_pub = rospy.Publisher('/gripping_state', Int8, queue_size=10)
		self.switch_pub = rospy.Publisher('/is_robot_moving', String, queue_size=10)
		self.lv_CSbUR_sub = rospy.Subscriber("length_vector2catch", Point, self.length_vecotr_CSbUR_update)

	def callback(self, point):
		if self.number_of_samplings_to_skip:
			self.number_of_samplings_to_skip -=1
		else:
			if self.counter < self.sample_count and not self.is_robot_moving:
				self.sample(point)
				self.counter += 1
			else:
				self.sample_succeed = True

			if self.sample_succeed and not self.is_robot_moving and self.gripping_step < 4:
				self.sample_counter += 1  # to count the samples
				print(str("---\nSampling " + str(self.sample_counter) + ". finished..."))
				# print("time: " + str(datetime.datetime.now()) + "\n")
				
				m_z, std_z = statistic_analysis(self.z_array)
				m_x, std_x = statistic_analysis(self.x_array)
				m_y, std_y = statistic_analysis(self.y_array)

				# conditions to lock the object
				# which means the object is found
				if object_locked(m_z, std_z):
					print("  Object locked...")
					# convert an array to a value
					goal_x = m_x[0]
					goal_y = m_y[0]
					goal_z = m_z[0]
					if self.end_of_profile2grasp:
						goal_x = m_x[0] + self.length_vecotr_CSbUR[0]
						goal_y = m_y[0] + self.length_vecotr_CSbUR[1]
						goal_z = m_z[0] + self.length_vecotr_CSbUR[2]
					euler_x = -3.1415
					euler_y = +0.0000
					euler_z = +0.0000
					# h_profil_midpoint = 30/2 + 2.5 # 2.5mm is the hegiht of plastic heimsphere
					# delta_h = 6.5 # 6.5mm is the difference of tcp's height when tcp is set according to  
					# the width of gripping finger 41.5mm (while gripping the profile successfully) and 60mm (default setting)
					# h_offset = -17 from experiments
					goal_z = goal_z + (17.5 + 6.5 - 17.0)/1000.0
					goal_tcp_pose = [goal_x, goal_y, goal_z, euler_x, euler_y, euler_z]
					if self.gripping_step == GrippingProcedure.nothing_done.value:
						print("    Translation and Orientation are in calculation...")
						goal_tcp_pose[2] += self.height_top_view
						current_pose = np.array(self.tcp_pose)
						pose_z_down = z_straight_down(current_pose)
						goal_tcp_pose[3] = pose_z_down[3]
						goal_tcp_pose[4] = pose_z_down[4]
						goal_tcp_pose[5] = pose_z_down[5]
						ur_move_str = "movel(p" + str(goal_tcp_pose) + ", a=0.2, v=0.2, t=0, r=0)"
						# print(goal_tcp_pose)
					elif self.gripping_step == GrippingProcedure.xy_position_done.value:
						print("    Rotation is in calculation...")
						# goal_tcp_pose = self.tcp_pose
						# gamma = self.gamma
						# goal_tcp_pose[5] += gamma
						# ur_move_str = "movel(p" + str(goal_tcp_pose) + ", a=0.2, v=0.05, t=0, r=0)"

						goal_joint_position = self.joint_position
						goal_joint_position_formated = [ float('%.2f' % elem) for elem in goal_joint_position]
						print("UR is going to rotate from joint position (rad):  {}".format(goal_joint_position_formated))
						gamma = self.gamma
						goal_joint_position[5] += gamma
						if goal_joint_position[5] > 5:
							goal_joint_position[5] = goal_joint_position[5] - 3.1415926 * 2
						elif goal_joint_position[5] < -5:
							goal_joint_position[5] = goal_joint_position[5] + 3.1415926 * 2
						goal_joint_position_formated = [ float('%.2f' % elem) for elem in goal_joint_position]
						print("                        to joint position (rad):  {}".format(goal_joint_position_formated))
						# print("gamma = "+str(gamma))
						# print(self.joint_position)
						# print(goal_joint_position)
						ur_move_str = "movej(" + str(goal_joint_position) + ", a=0.2, v=0.2, t=0, r=0)"
					elif self.gripping_step == GrippingProcedure.orientate_done.value:
						# keep the orientation unchanged and add offset to move into target point
						if not self.end_of_profile2grasp:
							goal_tcp_pose[0] += self.target_offset[0]
							goal_tcp_pose[1] += self.target_offset[1]
							goal_tcp_pose[2] += self.target_offset[2]  # not sure
						else:
							goal_tcp_pose[0] -= self.target_offset[0]*2
							goal_tcp_pose[1] -= self.target_offset[1]*0
							goal_tcp_pose[2] -= self.target_offset[2]  # not sure
						euler_reference = self.tcp_pose
						goal_tcp_pose[3] = euler_reference[3]
						goal_tcp_pose[4] = euler_reference[4]
						goal_tcp_pose[5] = euler_reference[5]
						# robot goes straight down and give a limit
						goal_tcp_pose[2] += (self.z_finger + self.z_offset) / 1000
						ur_move_str = "movel(p" + str(goal_tcp_pose) + ", a=0.2, v=0.1, t=0, r=0)"
					elif self.gripping_step == GrippingProcedure.z_position_done.value:
						# keep the orientation and xy-position unchanged
						goal_tcp_pose = self.tcp_pose
						if goal_tcp_pose[2] - (self.z_finger + self.z_offset) / 1000 < self.z_min / 1000:
							goal_tcp_pose[2] = self.z_min / 1000
						else:
							goal_tcp_pose[2] -= (self.z_finger + self.z_offset) / 1000
						# robot goes straight down and give a limit
						if self.end_of_profile2grasp: # grasping of 2. end requires a rotation arounf y-axis about -2.7 degree
							goal_tcp_pose[4] += -0.04712 #rad: -2.7/180.0*pi
							goal_tcp_pose[2] += 0.002 # vertical displacement dur to lifting						
						ur_move_str = "movel(p" + str(goal_tcp_pose) + ", a=0.2, v=0.05, t=0, r=0)"

					# safe limits to avoid crash into base or ground
					self.safe_flag = self.safety_checker(goal_tcp_pose[0], goal_tcp_pose[1], goal_tcp_pose[2])
					if self.gripping_step == 1:
						print("**Rotation angle: {:.4} degree".format(gamma / 3.1415926 * 180))
						self.safe_flag = True
					else:
						print("**Final target:[{:.4}, {:.4}, {:.4}] (m)".format(goal_x, goal_y, goal_z))

					if self.safe_flag:

						# manual_confirmation = input("Sure to move? (y/n): ")      # Python3
						# manual_confirmation = raw_input("Sure to move? (y/n): ")  # Python2
						print("Sure to move? (y/n):   (Please answer it within 3 seconds.)")
						self.is_robot_moving = True
						# print("stop sampling time: " + str(datetime.datetime.now()) + "\n")
						stop_sampling_time = datetime.datetime.now()

						# ## option 1----manual operation/check
						# i, o, e = select.select([sys.stdin], [], [], 2)  # timeout: 3s
						# if i:
						# 	manual_confirmation = sys.stdin.readline().strip()
						# 	print("Command received.")
						# else:
						# 	manual_confirmation = 'n'
						# 	print("No command. Next sampling is going on...")
						## option 2---- automatic operation
						manual_confirmation = 'y'
						# print("man :" + str(manual_confirmation))

						if manual_confirmation == 'y' and self.gripping_step <= 3:

							self.move_client.wait_for_server()
							goal = RobotMoveGoal()
							goal.move_command = ur_move_str
							self.move_client.send_goal(goal)
							self.move_client.wait_for_result()
							# self.move_client.wait_for_result(rospy.Duration.from_sec(10.0))
							# result = RobotMoveResult()
							result = self.move_client.get_result()
							print(result)
							if result.movement_finished:
								if self.gripping_step == 1:
									rospy.sleep(1.0)
								# self.is_robot_moving = False
							self.gripping_step += 1
							print("Finished: "+str(GrippingProcedure(self.gripping_step)))
						elif manual_confirmation == 'n':
							print("##Movement manually rejected--")
							# self.is_robot_moving = False
						
						elif self.gripping_step >= 4:
							print("##Gripping finished.")
						self.is_robot_moving = False
						# print("continue sampling time: " + str(datetime.datetime.now()) + "\n")
						continue_sampling_time = datetime.datetime.now()
						dtime_datetime = continue_sampling_time - stop_sampling_time
						dtime_seconds = dtime_datetime.seconds  + 1.0*dtime_datetime.microseconds/1000000 # no need to take 3s(waiting for input) into account?
						self.number_of_samplings_to_skip = int(dtime_seconds/(1.0/30)) # 30FPS
						# print("skip {} seconds samplings".format(int(dtime_seconds)))
						
						print("time interval for skipping sampling: {}".format(dtime_seconds))
						# print(number_of_samplings_to_skip)
					else:
						print("Dangerous to move! Movel cancelled!")
				else:
					print("  Searching object...")

				# initialization
				# prepare for a new sampling
				self.sample_succeed = False
				self.sampled_data_clean()
			if self.gripping_step == 4 and self.gripping_state_pub_counter > 0:
				self.gripping_state_pub_counter -=1
			if self.gripping_state_pub_counter > 0:
				self.gripping_state_pub.publish(self.gripping_step)

	def sample(self, point):
		self.x_array = np.append(self.x_array, point.x)
		self.y_array = np.append(self.y_array, point.y)
		self.z_array = np.append(self.z_array, point.z)

	def sampled_data_clean(self):
		self.x_array = []
		self.y_array = []
		self.z_array = []
		self.counter = 0	

	def safety_checker(self, x, y, z):
		x = x * 1000
		y = y * 1000
		z = z * 1000
		safe_x = False
		safe_y = False
		safe_z = False
		safe_flag = False
		if x < self.x_lowerlimit - self.xy_offset or x > self.x_upperlimit + self.xy_offset:
			safe_x = True
		if y < self.y_lowerlimit - self.xy_offset or y > self.y_upperlimit + self.xy_offset:
			safe_y = True
		if z >= self.z_finger + self.z_offset and not safe_x and not safe_y:
			safe_z = True
			safe_flag = True
		elif z >= self.z_ground:
			safe_z = True
			safe_flag = True
		else:
			safe_flag = False
		edge = 250  # mm : edge length of ur_base
		if x > -edge / 2 and x < edge / 2:
			if y > -edge / 2 and y < edge / 2:
				if z >= 0 and z < edge:
					safe_z = False
					safe_flag = False
		if safe_flag:
			print("      Yes!goal:[{:.4}, {:.4}, {:.4}] (m) is safe to go".format(x / 1000, y / 1000, z / 1000))
		else:
			print("      No! Not safe to move into [{:.4},{:.4},{:.4}] mm".format(x, y, z))
			print("          because of safe_flags x,y,z = " + str(int(safe_x)) + str(int(safe_y)) + str(int(safe_z)))
		return safe_flag

	def angle_update(self, angle):
		# print(angle.data)
		self.gamma = angle.data / 180 * 3.1415926

	def joint_state_update(self, joint_state):
		self.joint_position = list(joint_state.position)

	def tcp_pose_update(self, tcp_pose_msg):
		self.tcp_pose = list(tcp_pose_msg.tcp_pose)

	# print(self.tcp_pose)

	def target_offset_update(self, offset):
		self.target_offset = [offset.x, offset.y, offset.z]
	
	def length_vecotr_CSbUR_update(self, lv):
		self.length_vecotr_CSbUR = [lv.x, lv.y, lv.z]


def statistic_analysis(array):
	df = pd.DataFrame(array)
	series = df.describe()
	values = series.values
	median = values[5]
	# print("median = " + str(median))
	# # display the result
	# print(df.describe())
	# df.plot.box(title=str(self.sample_counter) + ". 5s data")
	# plt.grid(linestyle="--", alpha=0.3)
	# plt.show()
	std_deviation = values[2]
	return median, std_deviation


def object_locked(median, std):
	# output: x,y,z
	# judge dimensions
	if std < 15:
		return True
	else:
		return False


if __name__ == '__main__':

	rospy.init_node("goal_publisher")
	go = GraspObject()
	go.end_of_profile2grasp = False
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Stop tranfering the pose of goal object to robot!")
