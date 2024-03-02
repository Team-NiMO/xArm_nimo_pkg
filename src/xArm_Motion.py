import sys
import numpy as np
import rospy
from geometry_msgs.msg import Pose

#API 
from xarm.wrapper import XArmAPI

<<<<<<< Updated upstream
=======
#Perception
from stalk_detect.srv import GetStalk

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        self.arm.set_servo_angl
=======
        self.arm.set_servo_angle(angle=[0, -90, 0, 0, 0, 0], is_radian=False, wait=True)

    # default vertical stow position
    def vertical_stow_reset(self):
        print(f"---- going to vertical stow position ----")
        self.arm.reset(wait=False)

    # move to external mechanism plane
    def ext_mech_plane(self):
        print(f"---- going to external mechanisms plane position ----")
        # Original pose: self.arm.set_servo_angle(angle=[-87.8, 41.5, -40.3, 0, -88.3, 0], is_radian=False, wait=True)
        self.arm.set_servo_angle(angle=[-90, 41.5, -40.3, 0, -88.3, 0], is_radian=False, wait=True)
        # include another transform to account for the height of the height of the cornstalk
        
    # Cleaning and calibration
    def cleaning(self):
        print(f"---- going to cleaning nozzle ----")### TO DO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###
        self.arm.set_servo_angle(angle=[-133.6, 56.9, -53.4, 46, -85.6, 0], is_radian=False, wait=True)
        ### TO DO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###

    def calibration(self):
        print(f"---- going to calibration nozzles ----")
        self.arm.set_servo_angle(angle=[-118.3, 60.2, -69.9, 62.8, -77.9, -13.2], is_radian=False, wait=True)
        self.arm.set_servo_angle(angle=[-110.4, 75.9, -107.8, 73, -74.5, -33.8], is_radian=False, wait=True)
        ### TO DO: MODIFY THIS TO INCLUDE NEW CODE FROM AMIGA ###

    #######################################################
    # Functions that integrate with perception
    #######################################################
    
    #Identify the nearest cornstalk -- is this integrated within perception?? confirm 
    

    #Get the pose of the cornstalk that has been identified
    def stalk_pose (self):
        print(f" ---- getting stalk pose ----")

        rospy.wait_for_service('get_stalk')
        get_stalk_service = rospy.ServiceProxy('get_stalk', GetStalk)
        try:
            resp1 = get_stalk_service(num_frames=1, timeout=30.0) #5 frames,20 sec
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        print(' ---- Got response from stalk detection:', resp1.position)

        print("POSE IS", [resp1.position.x, resp1.position.y, resp1.position.z])
        return  np.array([resp1.position.x, resp1.position.y, resp1.position.z])
        
    # Move to nearest cornstalk (using the pose obtained above)
    
        
    # Arc motion of the end effector: Width detection

    #######################################################
    # Functions that integrate with end effector
    #######################################################

>>>>>>> Stashed changes
    

if __name__ == "__main__":
    print("------------------ testing main of xArm_Motion.py ------------------")