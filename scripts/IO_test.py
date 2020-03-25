#!/usr/bin/env python
# --------------------------------------
# file:      IO_test.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     set&read IO ports' state
#            bridge connection in UR
#            digital_output 7 <--> digital input 7 = program start
#            digital_output 6 <--> digital input 6 = program stop
#            digital_output 5 <--> digital input 5 = gripper open (width >85mm = analog input2>2.0, see Placing.txt)
#            digital_output 4 <--> digital input 4 = action on first (0) or second (1) end of the profile
#            details see Placing.txt
# --------------------------------------------------

import roslib
import rospy
import urx
import actionlib
from std_msgs.msg import String
from onrobot_rg2_gripper import OnRobotGripperRG2
from murc_robot.msg import RG2GripperAction, RG2GripperResult, RG2GripperFeedback
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

# # valid function values
# #
# # Note: 'fun' is short for 'function' (ie: the function the service should perform).
# int8 FUN_SET_DIGITAL_OUT = 1
# int8 FUN_SET_FLAG = 2
# int8 FUN_SET_ANALOG_OUT = 3
# int8 FUN_SET_TOOL_VOLTAGE = 4

# # valid values for 'state' when setting digital IO or flags
# int8 STATE_OFF = 0
# int8 STATE_ON = 1

# # valid 'state' values when setting tool voltage
# int8 STATE_TOOL_VOLTAGE_0V = 0
# int8 STATE_TOOL_VOLTAGE_12V = 12
# int8 STATE_TOOL_VOLTAGE_24V = 24
def  set_io_states_client(fun_value, pin_value, tool_voltage_value):
	rospy.wait_for_service("/ur_driver/set_io")
	try:
		client = rospy.ServiceProxy("/ur_driver/set_io", SetIO)
		response = client(fun_value, pin_value, tool_voltage_value)
		return response
	except rospy.ServiceException as e:
		print("Service call failed: "+str(e))

def analog_print_out(states_type):
	pins =   [item.pin for item in states_type]
	states = [item.state for item in states_type]
	print("pins:    "+str(pins))
	print("states:  "+str(states))

def digital_print_out(states_type):
	pins =   [str(item.pin) for item in states_type]
	states = [1 if item.state else 0 for item in states_type]
	states = [" "+str(item) if index > 9 else str(item) for index,item in enumerate(states)]
	print("pins:    "+str(pins))
	print("states:  "+str(states))

def readIOstates(io_states_msg):
	print("--------")
	digital_in_states = io_states_msg.digital_in_states
	digital_out_states = io_states_msg.digital_out_states
	flag_states = io_states_msg.flag_states	
	analog_in_states = io_states_msg.analog_in_states	
	analog_out_states = io_states_msg.analog_out_states	
	print("--digital_in_states:")
	digital_print_out(digital_in_states)
	print("--digital_out_states:")
	digital_print_out(digital_out_states)
	# print(digital_out_states[5].state)
	print("--flag_states:")
	print(flag_states)
	print("--analog_in_states:")
	analog_print_out(analog_in_states)
	print("--analog_out_states:")
	analog_print_out(analog_out_states)

# bridge connection in UR
# digital_output 7 <--> digital input 7 = program start
# difital_output 6 <--> digital input 6 = program stop  
def program_in_UR_start():
	set_io_states_client(1,6,0)
	set_io_states_client(1,7,1)

def program_in_UR_stop():
	set_io_states_client(1,7,0)
	set_io_states_client(1,6,1)

def reset_DO78():
	set_io_states_client(1,7,0)
	set_io_states_client(1,6,0)
	set_io_states_client(1,5,0)
	set_io_states_client(1,4,0) #first end placing DI4=DO4=0

def second_end_lifting():
	set_io_states_client(1,4,1) #second end lifting DI4=DO4=1


if __name__ == '__main__':
	rospy.init_node('io_test')
	# rospy.Subscriber("/ur_driver/io_states", IOStates, readIOstates)
	
	# performance = raw_input("Do you want to start the program in UR? (y=start,n=stop): ")
	# if performance == 'y':
	# 	program_in_UR_start()
	# elif performance == 'n':
	# 	program_in_UR_stop()

	reset_DO78()
	second_end_lifting()
	rospy.sleep(3.0)
	program_in_UR_start()
	rospy.spin()