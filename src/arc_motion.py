#!/usr/bin/env python3

import os
import sys
import time
import math
import argparse
import csv

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))


#API
from xarm.wrapper import XArmAPI
from configparser import ConfigParser

#custom helper library
import xArm_Motion as xArm_Motion

'''
###################################################################
xArm6 arc/circular movement around a centre point
###################################################################
'''

def get_args_parser():
    parser = argparse.ArgumentParser('xArm_motion', add_help=False)
    # relevant parameters
    parser.add_argument('--ip', default='192.168.1.214', type=str)
    return parser

def calculate_position(centre, radius, angle):
    x_t = centre[0] + radius * math.cos(math.radians(angle))
    y_t = centre[1] + radius * math.sin(math.radians(angle))
    z_t = centre[2]
    # return target parameters
    # print("x_t, y_t, z_t", x_t, y_t, z_t)
    return [x_t, y_t, z_t]

if __name__ == "__main__":
    print(" ================ start ============ ")
    parser = argparse.ArgumentParser('xArm_motion', parents=[get_args_parser()])
    args = parser.parse_args()
    
    # create xArm_Motion object
    xArm = xArm_Motion.xArm_Motion(args.ip) #SNOTE - creates an xarm wrapper with the ip passed through with the get_args_parser function
    xArm.initialize_robot()

    #print count down 3,2,1
    # for i in range(3,0,-1):
    #     print(f" starting in {i}... ")
    #     time.sleep(1)
    
    xArm.go_to_home()

    # [x,y,z] of the cornstalk position which is considered as the centre of the "circle"
    centre = [0.03628206015960567*1000,-0.1458352749289401*1000, 0.1441743369126926*1000]
    print("\ncentre:", centre)
    
    # print("centre[0]+29, centre[1]-32, centre[2]:\n",centre[0]+29, centre[1]-32, centre[2])
    print("centre[0], centre[1], centre[2]:\n",centre[0], centre[1], centre[2])
    

    print("\nmoving to the centre of the cornstalk")

    # xArm.arm.set_position_aa(axis_angle_pose=[centre[0]+29, centre[1]-32, centre[2], 0, 0, 0], relative=True, wait=True)
    xArm.arm.set_position_aa(axis_angle_pose=[centre[0], centre[1], centre[2], 0, 0, 0], relative=True, wait=True)
    print("\nxArm.arm.get_joint_states():",xArm.arm.get_joint_states())
    time.sleep(3)


    radius = 20 # in mm
    # define the start and end angles
    start = 0 
    end = 80
    angle_step = 1

    # '''
    ## Test for a single angle
    print("\nangle:", 50)
    pose_t = calculate_position(centre, radius, 50)
    print("\ntarget position:", pose_t)
    xArm.arm.set_position_aa(axis_angle_pose=[pose_t[0], pose_t[1], pose_t[2], 0, 0, 0], relative=True, wait=True)
    print("\nIK: ",xArm.arm.get_inverse_kinematics([pose_t[0], pose_t[1], pose_t[2], 0, 0, 0]))
    print("\nxArm.arm.get_joint_states():",xArm.arm.get_joint_states())
    time.sleep(3)
    # print("pose_t[0]+29, pose_t[1]-32, pose_t[2]\n", pose_t[0]+29, pose_t[1]-32, pose_t[2])
    print("\npose_t[0], pose_t[1], pose_t[2]:", pose_t[0], pose_t[1], pose_t[2])
    print("\n")
    # '''

    # fh = open('post_t.csv', 'w')
    # writer = csv.writer(fh)

    # writer.writerow(["angle", "x", "y"])
    # fh.close()


    '''
    for angle in range(start, end, angle_step):
        print("\nangle:", angle)
        pose_t = calculate_position(centre, radius, angle)
        print("\ntarget position:", pose_t)
        # xArm.arm.set_position_aa(axis_angle_pose=[pose_t[0]+29, pose_t[1]-32, pose_t[2], 0, 0, 0], relative=True, wait=True)

        # fh = open('pose_t.csv', 'a')  # `a` for `append mode`
        # writer = csv.writer(fh)
        # result = [angle, pose_t[0], pose_t[1]]
        # writer.writerow(result)
        # fh.close()

        print("\n",xArm.arm.set_position_aa(axis_angle_pose=[pose_t[0], pose_t[1], pose_t[2], 0, 0, 0], relative=True, wait=True))
        # time.sleep(3)
        # print("pose_t[0]+29, pose_t[1]-32, pose_t[2]\n", pose_t[0]+29, pose_t[1]-32, pose_t[2])
        print("\npose_t[0], pose_t[1], pose_t[2]:", pose_t)
        print("\n")
    '''

    # for angle in range(start, end, angle_step):
    #     pose_t = calculate_position(centre, radius/1000, angle)
    #     print("\nangle:", angle)
    #     print("\ntarget position:", pose_t )
    #     xArm.arm.set_position(x=pose_t[0], y=pose_t[1], z=pose_t[2])
    #     print("\n")
    
    # xArm.vertical_stow_reset()
    xArm.arm.disconnect()

    print(" ================ completed script ============ ")
