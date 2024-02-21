import sys
import numpy as np
import rospy
from geometry_msgs.msg import Pose

#API 
from xarm.wrapper import XArmAPI

class xArm_Motion():
    def __init__(self, ip_addr):
        print(f"---- creating xArm_Wrapper for ip {ip_addr} ----")
        self.ip = ip_addr
    
    def initialize_robot(self):
        print(f"---- initializing robot ----")
        self.arm = XArmAPI(self.ip)
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)

    # home position motion
    def go_to_home(self):
        print(f"---- going to home position ----")
        self.arm.set_servo_angl
    

if __name__ == "__main__":
    print("------------------ testing main of xArm_Motion.py ------------------")