#!/usr/bin/env python

import os
import sys
import time
import math
import argparse
import signal

# import roslib; roslib.load_manifest('smach_tutorials')

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
# xArm = xArm_Motion.xArm_Motion("192.168.1.196") # xArm6 IP address
# xArm.initialize_robot()

"""
States of the system -
High-level
State A: Finding Cornstalk -> State 1 -> Method 1 -> State 2 -> Method 2 -> State B ----> State Machine Outcome - STOP
State B: Cleaning -> State 1 -> State 3 -> Method 3 -> Method 5 -> Method 4 -> Method 5 -> Method 3 -> State C
State C: Insertion -> State 1 -> State 2 -> Method 5 -> Method 6 -> Method 7 -> Method 8 -> State 3 -> Method 3 ------> STOP

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
Method 8: Remove Sensor (retract linear actuator)
"""


def m_1 ():
    rospy.loginfo('Finding nearest Cornstalk')

def m_2 ():
    rospy.loginfo('Doing Width Detection')

def m_3 ():
    rospy.loginfo('Go to Clean Nozzle')

def m_4 ():
    rospy.loginfo('Go to Calibrate Nozzle')

def m_5 ():
    rospy.loginfo('Logging Data')

def m_6 ():
    rospy.loginfo('Desired Arc Movement')

def m_7 ():
    rospy.loginfo('Insert Sensor-Actuate Linear Actuator')

def m_8 ():
    rospy.loginfo('Remove Sensor-Retract Linear Actuator')


# State 0 - start
class state0(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['start'])
        
    def execute(self, userdata):
        rospy.loginfo('Running State 0')
        return 'start'

# State 1 - Home/Stow Position
class state1(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                            outcomes = ['s2','s3'],
                            input_keys = ['state_1_input'])
        
    def execute(self, userdata):
        print(userdata.state_1_input)
        rospy.loginfo('Running State 1')

        if userdata.state_1_input == 1:
            m_1()
            return 's2'
        elif userdata.state_1_input == 2:
            return 's3'
        elif userdata.state_1_input == 3:
            return 's2'
    
class state2(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['sb','s3'],
                             input_keys = ['state_2_input'])
    
    def execute(self, userdata):
        print(userdata.state_2_input)
        rospy.loginfo('Running State 2')

        if userdata.state_2_input == 1:
            m_2()
            return 'sb'
        elif userdata.state_2_input == 3:
            m_5()
            m_6()
            m_7()
            m_8()
            return 's3'

class state3(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['sc','s4'],
                             input_keys = ['state_3_input'])
    
    def execute(self, userdata):
        print(userdata.state_3_input)
        rospy.loginfo('Running State 3')

        if userdata.state_3_input == 2:
            m_3()
            m_5()
            m_4()
            m_5()
            m_3()
            return 'sc'
        elif userdata.state_3_input == 3:
            m_3()
            return 's4'

class state4(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['stop'],
                             input_keys = ['state_4_input'])
    
    def execute(self, userdata):
        print(userdata.state_4_input)
        rospy.loginfo('Running State 4')

        if userdata.state_4_input == 3:
            return 'stop'
        
def main():
    rospy.init_node('nimo_state_machine')

    Start_state = smach.StateMachine(outcomes = ['end'])    # Outcome of Main State Machine

    with Start_state:

        smach.StateMachine.add('State_0',state0(),
                               transitions = {'start':'State_A'})  # Go to State Machine A
        
        state_a_sm = smach.StateMachine(outcomes = ['sb_sm'])   # Outcome of State Machine A
        state_a_sm.userdata.flag_a = 1

        with state_a_sm:

            smach.StateMachine.add('State_1',state1(),
                                    transitions = {'s2':'State_2',
                                                  's3':'State_3'},
                                    remapping = {'state_1_input':'flag_a'})
            
            smach.StateMachine.add('State_2',state2(),
                                    transitions = {'sb':'sb_sm',
                                                  's3':'State_3'},
                                    remapping = {'state_2_input':'flag_a'})
            
        smach.StateMachine.add('State_A',state_a_sm,
                                transitions = {'sb_sm':'State_B'})    # State Machine A transitioning to State Machine B
            
        state_b_sm = smach.StateMachine(outcomes = ['sc_sm'])   # Outcome of State Machine B
        state_b_sm.userdata.flag_b = 2

        with state_b_sm:

            smach.StateMachine.add('State_1',state1(),
                                    transitions = {'s2':'State_2',
                                                    's3':'State_3'},
                                    remapping = {'state_1_input':'flag_b'})
            
                
            smach.StateMachine.add('State_3',state3(),
                                        transitions = {'sc':'sc_sm',
                                                    's4':'State_4'},
                                        remapping = {'state_3_input':'flag_b'})
            
        smach.StateMachine.add('State_B',state_b_sm,
                                transitions = {'sc_sm':'State_C'})    # State Machine B transitioning to State Machine C
        
        
        state_c_sm = smach.StateMachine(outcomes = ['finished'])    # Outcome of State Machine C
        state_c_sm.userdata.flag_c = 3

        with state_c_sm:
            
            smach.StateMachine.add('State_1',state1(),
                                    transitions = {'s2':'State_2',
                                                    's3':'State_3'},
                                    remapping = {'state_1_input':'flag_c'})
            
            smach.StateMachine.add('State_2',state2(),
                                    transitions = {'sb':'sb_sm',
                                                    's3':'State_3'},
                                    remapping = {'state_2_input':'flag_c'})
            
            smach.StateMachine.add('State_3',state3(),
                                    transitions = {'sc':'sc_sm',
                                                    's4':'State_4'},
                                    remapping = {'state_3_input':'flag_c'})
            
            smach.StateMachine.add('State_4',state4(),
                                    transitions = {'stop':'finished'},
                                    remapping = {'state_4_input':'flag_c'})
            
        smach.StateMachine.add('State_C',state_c_sm,                    
                                transitions = {'finished':'end'})     # State Machine C transitioning to the output of Main State Machine
    
            
    outcome = Start_state.execute()

if __name__ == '__main__':
    main()
