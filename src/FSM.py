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
xArm = xArm_Motion.xArm_Motion("192.168.1.196") # xArm6 IP address
xArm.initialize_robot()

