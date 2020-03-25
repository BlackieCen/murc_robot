#!/usr/bin/env python
# --------------------------------------
# file:      imagebased_grasping_smach4.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     state machine to launch the whole gripping strategy
# --------------------------------------------------

import rospy
import smach
import smach_ros
from ur_pose_maker import ur_pose_maker
from approach2object import Approach2Object
from std_msgs.msg import Int8
from grasp_object import GraspObject
from IO_test import program_in_UR_start, program_in_UR_stop, reset_DO78, second_end_lifting
from ur_msgs.msg import IOStates
from movealongobject_cmd import RelativeMovementMiR
from rg2_gripper_switcher import GripperController
from place_skateboard import PlaceSkateboard


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out01', 'out0i'])
        self.commands_received = False
        self.interruption_flag = False

    def execute(self, ud):
        rospy.loginfo("Waiting for commands")
        # to fill out
        rospy.sleep(5.0)
        self.commands_received = True
        if self.interruption_flag:
            return 'out0i'
        if self.commands_received:
            return 'out01'


class Searching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out10', 'out12', 'out11'])
        self.object_found = False
        self.searching_timeout = False
        self.pose_finished = False
        self.gripper_control_finished = False

    def execute(self, ud):
        rospy.loginfo("Searching the object")
        # do Searching
        pose_searching = ur_pose_maker()
        # without 0.5s, the msg will not reach the subscriber
        rospy.sleep(0.5)
        self.pose_finished = pose_searching.make_pose(1)
        while True:
            rospy.sleep(0.1)
            self.pose_finished = pose_searching.pose_finished
            if self.pose_finished:
                break
        # ensure the gripper is open
        gc = GripperController()
        # without 0.5s, the msg will not reach the subscriber
        rospy.sleep(0.5)
        self.gripper_control_finished = gc.open_gripper()
        while True:
            rospy.sleep(0.1)
            self.gripper_control_finished = gc.gripper_switch_finished
            if self.gripper_control_finished:
                break
        self.object_found = True
        if self.object_found:
            return 'out12'
        else:
            if self.searching_timeout:
                return 'out10'
            else:
                return 'out11'


# class Moving(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['out20', 'out23','out2f'])
#         self.destination_arrived = False
#         self.moving_timeout = False
#         self.movement_counter = 0

#     def execute(self, ud):
#         rospy.loginfo("Robot is moving")
#         # do moving
#         rospy.sleep(3.0)
#         self.destination_arrived = True
#         # do check timeout
#         if self.movement_counter >= 2:
#             return 'out2f'
#         if self.moving_timeout:
#             return 'out20'
#         if self.destination_arrived:
#             self.movement_counter += 1
#             return 'out23'
class MiR_Moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['2_out12'],output_keys=['mir_output'])
        self.destination_arrived = False
        self.counter = 0

    def execute(self, ud):
        self.counter += 1
        rospy.loginfo("MiR is moving")
        # do moving
        if self.counter == 1:  # to grasp the first end of profile
            a2o = Approach2Object()
            a2o.MiR_only_move_once = 1
            while True:
                rospy.sleep(1.0)
                self.destination_arrived = not a2o.MiR_only_move_once
                if self.destination_arrived:
                    break
        elif self.counter == 2:  # to graspe the second end of profile
            rm = RelativeMovementMiR()
            rm.displacement = [2.8, 0.3, 0.]
            self.destination_arrived = rm.RMove_execute()
        if self.destination_arrived:
            rospy.loginfo("goal reached")
        else:
            rospy.logerr("MiR didn't reach the goal!")
        # reset 
        self.destination_arrived = False
        # rospy.sleep(5.0)
        ud.mir_output = 'mir'
        return '2_out12'


class UR_Preparing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['2_out23', '2_out2ps'], input_keys=['ur_input'])
        # self.destination_arrived = False
        self.pose_finished = False
        self.skateboard_placed = False

    def execute(self, ud):
        rospy.loginfo("UR is moving")
        # do moving
        pose_grasping_preparation = ur_pose_maker()
        # without 0.5s, the msg will not reach the subscriber
        rospy.sleep(0.5)
        self.pose_finished = pose_grasping_preparation.make_pose(2)
        while True:
            rospy.sleep(0.1)
            self.pose_finished = pose_grasping_preparation.pose_finished
            if self.pose_finished:
                break
        # rospy.sleep(3.0)
        # self.destination_arrived = True
        if ud.ur_input == 'ps':
            self.skateboard_placed = True
        if not self.skateboard_placed:
            return '2_out2ps'
        else:
            return '2_out23'


class Object_in_view(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['2_out3f', '2_out3i'])
        self.object_in_view = False

    def execute(self, ud):
        rospy.loginfo("Checking if object is still in view")
        # do checking
        rospy.sleep(1.0)
        self.object_in_view = True
        if self.object_in_view:
            return '2_out3f'
        else:
            return '2_out3i'


class Placing_skateboard(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['2_outps2'], output_keys=['ps_output'])
        self.destination_arrived = False

    def execute(self, ud):
        rospy.loginfo("Placing skateboard")
        # do placing
        ps = PlaceSkateboard()
        ps.ps_only_move_once = 1
        while True:
            rospy.sleep(1.0)
            self.destination_arrived = not ps.ps_only_move_once
            if self.destination_arrived:
                break

        if self.destination_arrived:
            rospy.loginfo("UR returned to gripping station.")
        else:
            rospy.logerr("UR didn't return to gripping station!")
        # reset
        self.destination_arrived = False
        ud.ps_output = 'ps'
        return '2_outps2'


# class Gripping(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['out34','out3f'])
#         self.gripping_finished = False
#         self.gripping_counter = 0

#     def execute(self, ud):
#         rospy.loginfo("Gripping the object")
#         # do Gripping
#         rospy.sleep(4.0)
#         self.gripping_finished = True
#         if self.gripping_finished:
#             self.gripping_counter += 1
#             if self.gripping_counter >= 2:
#                 return 'out3f'
#             else:
#                 return 'out34'
class xy_Positioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['3_out12'])
        self.gripping_step = 0
        self.gripping_counter = 0

    def execute(self, ud):
        rospy.loginfo("xy Positioning")
        # do xy Positioning
        gripping = GraspObject()
        if self.gripping_counter == 1:
            gripping.end_of_profile2grasp = True  # graspe the back end of profile
        while True:
            rospy.sleep(1.0)
            gripping_state_msg = rospy.wait_for_message('/gripping_state', Int8)
            self.gripping_step = gripping_state_msg.data
            print(gripping_state_msg)
            # self.gripping_step += 1
            if self.gripping_step == 1:
                break
        rospy.sleep(3.0)
        self.gripping_counter += 1
        return '3_out12'


class Orientating(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['3_out23'])
        self.gripping_step = 1

    def execute(self, ud):
        rospy.loginfo("Orientating to the object")
        # do Orientating
        while True:
            rospy.sleep(1.0)
            gripping_state_msg = rospy.wait_for_message('/gripping_state', Int8)
            self.gripping_step = gripping_state_msg.data
            # self.gripping_step += 1
            if self.gripping_step == 2:
                break
        rospy.sleep(3.0)
        return '3_out23'


class z_Positioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['3_out34', '3_out35'])
        self.gripping_step = 2
        self.gripping_counter = 0

    def execute(self, ud):
        rospy.loginfo("z Positioning")
        # do z Positioning
        while True:
            rospy.sleep(1.0)
            gripping_state_msg = rospy.wait_for_message('/gripping_state', Int8)
            self.gripping_step = gripping_state_msg.data
            # self.gripping_step += 1
            if self.gripping_step == 4:  # z_positioning consists of 2 steps (step3 and step4)
                break
        rospy.sleep(3.0)
        self.gripping_counter += 1
        if self.gripping_counter >= 2:
            return '3_out35'
        else:
            return '3_out34'


class Placing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['4_out12'])
        self.placing_finished = False

    def execute(self, ud):
        rospy.loginfo("Placing the object")
        # do Placing
        reset_DO78()
        rospy.sleep(3.0)
        program_in_UR_start()
        while True:
            rospy.sleep(1.0)
            io_states_msg = rospy.wait_for_message('/ur_driver/io_states', IOStates)
            # digital output[5] is going to be set to True in program in the teach pendant, when program finishs
            self.placing_finished = io_states_msg.digital_out_states[5].state
            if self.placing_finished == True:
                break
        rospy.sleep(5.0)
        program_in_UR_stop()
        self.placing_finished = True
        if self.placing_finished:
            return '4_out12'


class Withdrawing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['4_out2f'])
        # self.destination_arrived = False
        self.pose_finished = False

    def execute(self, ud):
        rospy.loginfo("UR is returning")
        # do moving
        pose_return = ur_pose_maker()
        # without 0.5s, the msg will not reach the subscriber
        rospy.sleep(0.5)
        self.pose_finished = pose_return.make_pose(0)
        while True:
            rospy.sleep(0.1)
            self.pose_finished = pose_return.pose_finished
            if self.pose_finished:
                break
        # rospy.sleep(3.0)
        # self.destination_arrived = True
        return '4_out2f'


class LiftingandGo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out4f'])
        self.lifting_finished = False

    def execute(self, ud):
        rospy.loginfo("UR is lifting object")
        # do lifting
        reset_DO78()
        second_end_lifting()
        rospy.sleep(3.0)
        program_in_UR_start()
        while True:
            rospy.sleep(1.0)
            io_states_msg = rospy.wait_for_message('/ur_driver/io_states', IOStates)
            # digital output[5] is going to be set to True in program in the teach pendant, when program finishs
            self.lifting_finished = io_states_msg.digital_out_states[5].state
            if self.lifting_finished == True:
                break
        rospy.sleep(5.0)
        program_in_UR_stop()
        self.lifting_finished = True
        if self.lifting_finished:
            return 'out4f'


if __name__ == '__main__':
    rospy.init_node("imagebased_grasping_state_machine")

    sm = smach.StateMachine(outcomes=['finished', 'interrupted'])

    with sm:
        smach.StateMachine.add('Z0_IDLE', Idle(), transitions={'out01': 'Z1_SEARCHING',
                                                               'out0i': 'interrupted'})
        smach.StateMachine.add('Z1_SEARCHING', Searching(), transitions={'out10': 'Z0_IDLE',
                                                                         'out12': 'Z2_MOVING',
                                                                         'out11': 'Z1_SEARCHING'})
        # smach.StateMachine.add('Z2_MOVING', Moving(), transitions={'out20': 'Z0_IDLE',
        #                                                            'out23': 'Z3_GRIPPING',
        #                                                            })
        sm_Z2 = smach.StateMachine(outcomes=['out23', 'out20'])
        with sm_Z2:
            smach.StateMachine.add('Z21_MIR_MOVING', MiR_Moving(), transitions={'2_out12': 'Z22_UR_PREPARING'},
                                   remapping={'mir_output': 'ur_input'})
            smach.StateMachine.add('Z22_UR_PREPARING', UR_Preparing(), transitions={'2_out23': 'Z23_OBJECT_IN_VIEW',
                                                                                    '2_out2ps': 'Z2PS_Placing_Skateboard'},
                                   remapping={'ur_input': 'ur_input'})
            smach.StateMachine.add('Z23_OBJECT_IN_VIEW', Object_in_view(), transitions={'2_out3f': 'out23',
                                                                                        '2_out3i': 'out20'})
            smach.StateMachine.add('Z2PS_Placing_Skateboard', Placing_skateboard(),
                                   transitions={'2_outps2': 'Z22_UR_PREPARING'}, remapping={'ps_output': 'ur_input'})
        smach.StateMachine.add('Z2_MOVING', sm_Z2, transitions={'out20': 'Z0_IDLE',
                                                                'out23': 'Z3_GRIPPING'})

        # smach.StateMachine.add('Z3_GRIPPING', Gripping(), transitions={'out34': 'Z4_PLACING',
        #                                                                'out3f': 'finished'})
        sm_Z3 = smach.StateMachine(outcomes=['out34', 'out35'])
        with sm_Z3:
            smach.StateMachine.add('Z31_XY_POSITIONING', xy_Positioning(), transitions={'3_out12': 'Z32_ORIENTATING'})
            smach.StateMachine.add('Z32_ORIENTATING', Orientating(), transitions={'3_out23': 'Z33_Z_POSITIONING'})
            smach.StateMachine.add('Z33_Z_POSITIONING', z_Positioning(), transitions={'3_out34': 'out34',
                                                                                      '3_out35': 'out35'})
        smach.StateMachine.add('Z3_GRIPPING', sm_Z3, transitions={'out34': 'Z4_PLACING',
                                                                  'out35': 'Z5_LIFTINGANDGO'})
        # smach.StateMachine.add('Z4_PLACING', Placing(), transitions={'out42': 'Z2_MOVING'})
        sm_Z4 = smach.StateMachine(outcomes=['out42'])
        with sm_Z4:
            smach.StateMachine.add('Z41_PLACING', Placing(), transitions={'4_out12': 'Z42_WITHDARWING'})
            smach.StateMachine.add('Z42_WITHDARWING', Withdrawing(), transitions={'4_out2f': 'out42'})
        smach.StateMachine.add('Z4_PLACING', sm_Z4, transitions={'out42': 'Z2_MOVING'})

        smach.StateMachine.add('Z5_LIFTINGANDGO', LiftingandGo(), transitions={'out4f': 'finished'})

    sis = smach_ros.IntrospectionServer('introspection_server', sm, '/Imagebased_Grasping_SM')
    sis.start()

    outcome = sm.execute()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop state machine!")
    sis.stop()
