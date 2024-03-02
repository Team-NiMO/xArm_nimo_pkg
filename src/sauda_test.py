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
class test:

    def __init__ (self):
        self.outcomes_1 = ['outcome1','outcome2']
        self.outcomes_2 = ['outcome1']

        self.transitions = {
            'foo': {self.outcomes_1[0] : 'BAR', self.outcomes_1[1] : 'outcome4'},
            'bar': {self.outcomes_2[0] : 'FOO'}
        }

    # define state Foo
    class Foo(smach.State):
        def __init__(self):
            smach.State.__init__(self, 
                                outcomes=test().outcomes_1,
                                input_keys=['foo_counter_in'],
                                output_keys=['foo_counter_out'])

        def execute(self, userdata):
            # rospy.loginfo('Executing state FOO')
            # print(userdata.foo_counter_in)
            if userdata.foo_counter_in < 3:
                userdata.foo_counter_out = userdata.foo_counter_in + 1
                return test().outcomes_1[0]
            else:
                return test().outcomes_1[1]


    # define state Bar
    class Bar(smach.State):
        def __init__(self):
            smach.State.__init__(self, 
                                outcomes=test().outcomes_2,
                                input_keys=['bar_counter_in'])
            
        def execute(self, userdata):
            # rospy.loginfo('Executing state BAR')
            # rospy.loginfo('Counter = %f'%userdata.bar_counter_in)        
            return test().outcomes_2[0]
            

    def main(self):
        # rospy.init_node('smach_example_state_machine')

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['outcome4'])
        sm.userdata.sm_counter = 0

        # Open the container
        with sm:
            # Add states to the container

            smach.StateMachine.add('FOO', test.Foo(), 
                                transitions=self.transitions['foo'],
                                remapping={'foo_counter_in':'sm_counter', 
                                            'foo_counter_out':'sm_counter'})
            smach.StateMachine.add('BAR', test.Bar(), 
                                transitions=self.transitions['bar'],
                                remapping={'bar_counter_in':'sm_counter'})


        # Execute SMACH plan
        outcome = sm.execute()


if __name__ == '__main__':
    t = test()
    t.main()

