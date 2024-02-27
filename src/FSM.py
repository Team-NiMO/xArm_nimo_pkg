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

class State_Machine:

    def __init__ (self):
   
        self.outcomes_s_1 = ['s2_move_to_near_cs','s3_ext_mech']
        self.outcomes_s_2 = ['sb_cleaning','s3_ext_mech']
        self.outcomes_s_3 = ['s4_replace','sc_insertion']
        self.outcomes_s_4 = ['EXIT']

        self.transitions = {
            'state_1': {self.outcomes_s_1[0] : 'state_2', self.outcomes_s_1[1] : 'state_3'},
            'state_2': {self.outcomes_s_2[0] : 'state_B', self.outcomes_s_2[1] : 'state_3'},
            'state_3': {self.outcomes_s_3[0] : 'state_4', self.outcomes_s_3[1] : 'state_C'},
            'state_4': {self.outcomes_s_4[0] : 'STOP'}
        }

    def m_1 (self):
        rospy.loginfo('Finding nearest Cornstalk')

    def m_2 (self):
        rospy.loginfo('Doing Width Detection')

    def m_3 (self):
        rospy.loginfo('Go to Clean Nozzle')
    
    def m_4 (self):
        rospy.loginfo('Go to Calibrate Nozzle')

    def m_5 (self):
        rospy.loginfo('Logging Data')

    def m_6 (self):
        rospy.loginfo('Desired Arc Movement')
    
    def m_7 (self):
        rospy.loginfo('Insert Sensor-Actuate Linear Actuator')
    
    def m_8 (self):
        rospy.loginfo('Remove Sensor-Retract Linear Actuator')

    # State 1 - Home/Stow Position
    class state_1(smach.State):

        def __init__(self):
            smach.State.__init__(self, 
                                outcomes = State_Machine().outcomes_s_1,
                                input_keys = ['state_1_input'])

        def execute(self, userdata):
            rospy.loginfo('Executing state 1')
            if userdata.state_1_input == 1:
                State_Machine().m_1()
                return State_Machine().outcomes_s_1[0]
            elif userdata.state_1_input == 2:
                State_Machine().m_2()
                return State_Machine().outcomes_s_1[1]

    # State 2 - Move to nearest CornStalk
    class state_2(smach.State):
        def __init__(self):
            smach.State.__init__(self, 
                                outcomes = State_Machine().outcomes_s_2,
                                input_keys = ['state_2_input'])

        def execute(self, userdata):
            rospy.loginfo('Executing state 2')
            if userdata.state_2_input == 1:
                State_Machine().m_2()
                return State_Machine().outcomes_s_2[0]
            elif userdata.state_2_input == 3:
                State_Machine().m_5()
                State_Machine().m_6()
                State_Machine().m_7()
                State_Machine().m_8()
                return State_Machine().outcomes_s_2[1]
    
    # State 3 - Go to External Mechanisms
    class state_3(smach.State):
        def __init__(self):
            smach.State.__init__(self, 
                                outcomes = State_Machine().outcomes_s_3,
                                input_keys = ['state_3_input'])

        def execute(self, userdata):
            rospy.loginfo('Executing state 3')
            if userdata.state_3_input == 2:
                State_Machine().m_3()
                State_Machine().m_5()
                State_Machine().m_4()
                State_Machine().m_5()
                State_Machine().m_3()
                return State_Machine().outcomes_s_3[1]
            elif userdata.state_3_input == 3:
                State_Machine().m_3()
                return State_Machine().outcomes_s_3[0]
            
    # State 4 - Replace Sensor
    class state_4(smach.State):
        def __init__(self):
            smach.State.__init__(self, 
                                outcomes = State_Machine().outcomes_s_4,
                                input_keys = ['state_4_input'])

        def execute(self, userdata):
            rospy.loginfo('Executing state 4')
            if userdata.state_4_input == 3:
                return State_Machine().outcomes_s_4[0]
            

    def main(self):
        rospy.init_node('xarm_state_machine')

        # Create the top level SMACH state machine
        state_A = smach.StateMachine(outcomes=['STOP'])
        state_A.userdata.flag_A = 1    # Setting Flag = 1 for State A
        
        # Open the container
        with state_A:
            # Add States to State A
            
            smach.StateMachine.add('state_1', State_Machine.state_1(),
                                    transitions = self.transitions['state_1'],
                                    remapping = {'state_1_input':'flag_A'})
            
            smach.StateMachine.add('state_2', State_Machine.state_2(),
                                    transitions = self.transitions['state_2'][self.outcomes_s_2[0]],    # Outcome: State B
                                    remapping = {'state_2_input':'flag_A'})

            # Create the sub SMACH state machine
            state_B = smach.StateMachine(outcomes=['state_C'])
            state_B.userdata.flag_B = 2    # Setting Flag = 2 for State B

            # Open the container
            with state_B:
                # Add States to State B
                smach.StateMachine.add('state_1', State_Machine.state_1(),
                                    transitions = self.transitions['state_1'],
                                    remapping = {'state_1_input':'flag_B'})

                smach.StateMachine.add('state_3', State_Machine.state_3(),
                                    transitions = self.transitions['state_3'],
                                    remapping = {'state_3_input':'flag_B'})
                
                # Create the sub SMACH state machine
                state_C = smach.StateMachine(outcomes=['STOP'])
                state_C.userdata.flag_C = 3    # Setting Flag = 3 for State C
                
                # Open the container
                with state_C:
                    # Add State to State C
                    smach.StateMachine.add('state_1', State_Machine.state_1(),
                                    transitions = self.transitions['state_1'],
                                    remapping = {'state_1_input':'flag_C'})
                    
                    smach.StateMachine.add('state_2', State_Machine.state_2(),
                                    transitions = self.transitions['state_2'],
                                    remapping = {'state_2_input':'flag_C'})
                    
                    smach.StateMachine.add('state_3', State_Machine.state_3(),
                                    transitions = self.transitions['state_3'],
                                    remapping = {'state_3_input':'flag_C'})
                    
                    smach.StateMachine.add('state_4', State_Machine.state_4(),
                                    transitions = self.transitions['state_4'],
                                    remapping = {'state_4_input':'flag_C'})
                
                # Add state C to state B
                smach.StateMachine.add('state_C',state_C, transitions = self.transitions['state_4'])
                
            # Add state B to state A
            smach.StateMachine.add('state_B',state_B, transitions = self.transitions['state_4'])   # STOP

        # Execute SMACH plan
        outcome = state_A.execute()

if __name__ == '__main__':
    sm = State_Machine()
    sm.main()

