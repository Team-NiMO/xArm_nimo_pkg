import os
import sys
import time
import math
import argparse

# ROS
import rospy
import geometry_msgs.msg 
import tf2_ros

# FSM
import smach
import smach_ros


# API
from xarm.wrapper import XArmAPI
from configparser import ConfigParser

# custom helper library
import xArm_Motion as xArm_Motion

def get_args_parser():
    parser = argparse.ArgumentParser('xArm_motion', add_help=False)
    # relevant parameters
    parser.add_argument('--ip', default='192.168.1.214', type=str)
       
    return parser

# def REQ_DETECT():


if __name__ == "__main__":
    print(" ================ start ============ ")
    parser = argparse.ArgumentParser('xArm_motion', parents=[get_args_parser()])
    args = parser.parse_args()
    
    # create xArm_Motion object
    xArm = xArm_Motion.xArm_Motion(args.ip) #SNOTE - creates an xarm wrapper with the ip passed through with the get_args_parser function
    xArm.initialize_robot()

    #print count down 3,2,1
    for i in range(3,0,-1):
        print(f" starting in {i}... ")
        time.sleep(1)

    ## REQ DETECT BASE CODE:
    rospy.loginfo("Executing the state REQ_DETECT")
    detected_stalk_pose = xArm.stalk_pose()
    


    
    xArm.arm.disconnect()


    print(" ================ completed script ============ ")