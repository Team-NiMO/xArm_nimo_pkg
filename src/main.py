import os
import sys
import time
import math
import argparse

# ROS
import rospy

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

    # xArm.go_to_home()
    xArm.vertical_stow_reset()
    # print(f"move to external mechanisms plane")
    # xArm.arm.set_servo_angle(angle=[-90, 50, 0, 0, 0, 0], is_radian=False, wait=True)
    # xArm.ext_mech_plane()
    xArm.arm.disconnect()


    print(" ================ completed script ============ ")