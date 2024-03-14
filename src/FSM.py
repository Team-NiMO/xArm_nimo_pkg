import os
import sys
import time
import math
import argparse
import signal

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
sensor = False

def m_1 ():
    rospy.loginfo('Finding nearest Cornstalk')

def m_2 ():
    rospy.loginfo('Doing Width Detection')

def m_3 (self, flag):
    print(flag)
    rospy.loginfo('Go to Clean Nozzle')
    if (flag == 1 or flag == 2):
        if (not sensor):
            return False
        else:
            return True

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

def m_9 ():
    rospy.loginfo('Move arm to the nearest cornstalk')

def m_10 ():
    rospy.loginfo('Stow Position')

def m_11 ():
    rospy.loginfo('Replace Sensor')


# State 1 - Finding Cornstalk
class state1(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['cleaning_calibrating'])
        
    def execute(self, userdata):
        rospy.loginfo('Running State 1')
        m_10()
        m_1()
        m_9()
        m_2()
        return 'cleaning_calibrating'

# State 2 - Cleaning and Calibrating
class state2(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                            outcomes = ['insertion','replace'],
                            input_keys = ['c_c_ip'])    #outcome2: Replace
        
    def execute(self, userdata):
        rospy.loginfo('Running State 2')
        m_10()
        m_5()
        # decision_1 = m_3(userdata.c_c_ip)
        m_5()
        m_4()
        decision_2 = m_3(self, userdata.c_c_ip)
        if not decision_2:
            return 'replace'
        else:
            return 'insertion'
    
# State 3: Insertion
class state3(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['replace'],
                             input_keys = ['insert_ip'])
    
    def execute(self, userdata):
        rospy.loginfo('Running State 3')
        m_10()
        m_9()
        m_5()
        m_6()
        m_7()
        m_8()
        decision = m_3(self, userdata.insert_ip)
        if not decision:
            return 'replace'
    
# State 4: Replace
class state4(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['replace_stop'])
    
    def execute(self, userdata):
        rospy.loginfo('Running State 3')
        m_10()
        m_9()
        m_5()
        m_6()
        m_7()
        m_8()
        m_3()
        return 'replace_stop'

        
def main():
    rospy.init_node('nimo_state_machine')

    start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
    start_state.userdata.flag_a = 1
    start_state.userdata.flag_b = 2
    start_state.userdata.flag_c = 3

    with start_state:

        smach.StateMachine.add('Finding_Cornstalk',state1(),
                               transitions = {'cleaning_calibrating':'Cleaning_Calibrating'})  # Go to State B
        
        smach.StateMachine.add('Cleaning_Calibrating',state2(),
                               transitions = {'insertion':'Insertion','replace':'stop'},
                               remapping = {'c_c_ip':'flag_b'})  # Go to State B
        
        smach.StateMachine.add('Insertion',state3(),
                               transitions = {'replace':'stop'},
                               remapping = {'insert_ip':'flag_c'})  # Go to State B
    
    sis = smach_ros.IntrospectionServer('server_name', start_state, '/NiMo_SM')
    sis.start()

    outcome = start_state.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
