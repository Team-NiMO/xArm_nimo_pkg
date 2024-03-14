#!/usr/bin/env python3 

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

    # #print count down 3,2,1
    # for i in range(3,0,-1):
    #     print(f" starting in {i}... ")
    #     time.sleep(1)

    ## Hook identified cornstalks:
    rospy.loginfo("Executing the hard coded hook motion")

    x = -0.03628206015960567
    y = -0.1458352749289401
    z = -0.1441743369126926

    x = -x #gripper x coord is opposite direction of robot base x coord
    z = -z #gripper x coord is opposite direction of robot base x coord 

    xArm.go_to_home()
    print(xArm.arm.get_position)
    # xArm.go_to_plane()

    # xArm.go_to_stalk_pose(x*1000, y*1000, z*1000)

    # time.sleep(1)
    # xArm.go_to_home()
    # xArm.go_to_plane()

    # x = -0.05
    # y = -0.2
    # z = -0.05

    # xArm.go_to_stalk_pose(x*1000, y*1000, z*1000)

    # xArm.go_to_plane()
    # xArm.go_to_home()

    # xArm.arm.set_position(412, -48.1, 267.7, 180, 0, 0.2, is_radian=None)
    # FK = xArm.arm.get_joint_states()

    # xArm.vertical_stow_reset()

    # pose = [412, -48.1, 267.7, 180, 0, 0.2]
    # pose2 = [412.6, -48.5, 267.5, 180, 0, 0]

    # IK = xArm.arm.get_inverse_kinematics(pose)
    # IK2 = xArm.arm.get_inverse_kinematics(pose2)


    # print("\n\nFK:", FK)
    # print("\n\nIK:", IK)
    # print("\n\nIK2:", IK2)
    # xArm.arm.set_position(412, -48.1, 267.7, 180, 0, 0.2, is_radian=None)
    # xArm.arm.set_servo_angle(angle=pose, is_radian=None,wait=True)

    
    xArm.arm.disconnect()


    print(" ================ completed script ============ ")