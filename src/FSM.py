#!/usr/bin/env python

import os
import sys
import time
import math
import argparse
import signal

import roslib; roslib.load_manifest('smach_tutorials')

# ROS
import rospy
import tf2_ros
import geometry_msgs.msg

# FSM
import smach
import smach_ros

# custom helper library
import xArm_Motion as xArm_Motion
# import utils_plot as fsm_plot

# Global terms:
xArm = xArm_Motion.xArm_Motion("192.168.1.196") # xArm6 IP address
xArm.initialize_robot()

"""
States of the system -
High-level
State A: Finding Cornstalk -> State 1 -> Method 1 -> State 2 -> Method 2 -> State B
State B: Cleaning -> State 1 -> State 3 -> Method 3 -> Method 5 -> Method 4 -> Method 5 -> Method 3 -> State 4/State C
State C: Insertion -> State 1 -> State 2 -> Method 5 -> Method 6 -> Method 7 -> Method 8 -> State 3 -> Method 3 -> State 4/STOP/State A

Low-level
State 1: Home/Stow Position
State 2: Moving to the nearest cornstalk
State 3: Go to External Mechanisms - Clean + Calibrate (To a point which is just away from the mechanism)
State 4: Replace Sensor

Methods - 
Method 1: Finding nearest cornstalk - o/p: corn pose
Method 2: Width Detection for finding suitable side to insert (arc movement)- o/p: corn pose
Method 3: Go to Clean Nozzle
Method 4: Go to Calibrate Nozzle
Method 5: Log Data
Method 6: Desired Arc Movement
Method 7: Insert Sensor (actuate linear actuator)
Method 8: Remove Sensor (Retract linear actuator)
"""

class State_Machine:

    def __init__ (self):
   
        self.outcomes_s_1 = ['s3_ext_mech']
        self.outcomes_s_2 = ['s1_cleaning']

        self.transitions = {
            'state_1': {self.outcomes_s_1[0] : 'state_2'},
            'state_2': {self.outcomes_s_2[0]: 'state_1'}
        }

        def m_1 (self):
            rospy.loginfo('Finding nearest Cornstalk')

        def m_2 (self):
            rospy.loginfo('Doing Width Detection')

        def m_5 (self):
            rospy.loginfo('Logging Data')

        # State 1 - Home/Stow Position
        class state_1(smach.State):

            def __init__(self):
                smach.State.__init__(self, 
                                    outcomes = self.outcomes_s_1,
                                    input_keys = ['state_1_input'])

            def execute(self, userdata):
                rospy.loginfo('Executing state 1')
                if userdata.state_1_input == 1:
                    State_Machine.m_1()
                    return self.outcomes_s_1[0]

        # State 2 - Move to nearest CornStalk
        class state_2(smach.State):
            def __init__(self):
                smach.State.__init__(self, 
                                    outcomes = self.outcomes_s_2,
                                    input_keys = ['state_2_input'])

            def execute(self, userdata):
                rospy.loginfo('Executing state 2')
                if userdata.state_2_input == 1:
                    State_Machine.m_2()
                    return self.outcomes_s_2[0]
            
        self.state_1 = state_1
        self.state_2 = state_2

    def main(self):
        rospy.init_node('xarm_state_machine')

        # Create the top level SMACH state machine
        state_A = smach.StateMachine(outcomes=['stateB'])
        state_A.userdata.flag_A = 1    # Setting Flag = 1 for State A
        
        # Open the container
        with state_A:

            # Add States to State A
            smach.StateMachine.add('state_1', self.state_1(),
                                    transitions = self.transitions['state_1'],
                                    remapping = {'state_1_input':'flag_A'})
            
            smach.StateMachine.add('state_2', self.state_2(),
                                transitions = self.transitions['state_2'],
                                remapping = {'state_2_input':'flag_A'})

            # # Create the sub SMACH state machine
            # state_B = smach.StateMachine(outcomes=['outcomestateC'])

            # # Open the container
            # with state_B:

            #     # Add states to the container
            #     smach.StateMachine.add('FOO', Foo(), 
            #                         transitions={'outcome1':'BAR', 
            #                                         'outcome2':'outcome4'})
            #     smach.StateMachine.add('BAR', Bar(), 
            #                         transitions={'outcome1':'FOO'})

            # smach.StateMachine.add('SUB', sm_sub,
            #                     transitions={'outcome4':'outcome5'})

        # Execute SMACH plan
        outcome = state_A.execute()

if __name__ == '__main__':
    sm = State_Machine()
    sm.main()

