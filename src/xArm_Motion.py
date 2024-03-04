import sys
import numpy as np
import rospy
from geometry_msgs.msg import Pose

#API 
from xarm.wrapper import XArmAPI

#Perception
from stalk_detect.srv import GetStalk

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


    ##### TEST: MOVING TO THE STALK POSE #####
    def go_to_stalk_pose(self, x_mm,y_mm,z_mm):
        print(f"now do the APPROACH MOTION")
        
        x_mm_tuned_offset = 29

        x_mm_gripper_width = 80+5 #80mm is roughly center of gripper to edge of C clamp, 5 is fine-tuned offset
        x_mm_with_gripper_offest = x_mm + x_mm_gripper_width + x_mm_tuned_offset

        y_mm_tuned_offset = -32
        print(f"x_mm, x_mm_with_gripper_offest {x_mm, x_mm_with_gripper_offest}")

        
        # x_mm_deeper_clamp_width = 18
        x_mm_deeper_clamp_insert = 14
        x_mm_deeper_clamp_retract = 25

        z_mm_tuned = z_mm

        y_mm_overshoot = 8
        y_mm_funnel = 12

        print(f"y_mm+y_mm_tuned_offset {y_mm+y_mm_tuned_offset}")
        print(f"y_mm+y_mm_tuned_offset+ y_mm_overshoot {y_mm+y_mm_tuned_offset+y_mm_overshoot}")


        print(f" ---- going to stalk pose  ----")

        print(f" 1. move X align 1/10 ")
        # self.arm.set_position_aa(axis_angle_pose=[x_mm_with_gripper_offest, 0, 0, 0, 0, 0], relative=True, wait=True)
        print(f"x_mm_with_gripper_offest: {x_mm_with_gripper_offest}")
        # self.arm.set_position_aa(axis_angle_pose=[x_mm_with_gripper_offest, y_mm+y_mm_tuned_offset+y_mm_overshoot, z_mm_tuned, 0, 0, 0], relative=True, wait=True)
        print(f" 2. move Y approach  2/10")
        # self.arm.set_position_aa(axis_angle_pose=[0, y_mm+y_mm_tuned_offset, 0, 0, 0, 0], relative=True, wait=True)
        print(f"y_mm+y_mm_tuned_offset: {y_mm+y_mm_tuned_offset}")
        
        print(f" 2.5 move Y to compensate overshoot  2.5/10")
        # self.arm.set_position_aa(axis_angle_pose=[0, y_mm_overshoot, 0, 0, 0, 0], relative=True, wait=True)
        print(f"y_mm_overshoot: {y_mm_overshoot}")
        
        print(f" 3. move Z to down 3/10 with z: {z_mm_tuned}")
        # self.arm.set_position_aa(axis_angle_pose=[0, 0, z_mm_tuned, 0, 0, 0], relative=True, wait=True)
        print(f"z_mm_tuned: {z_mm_tuned}")

        print(f" 4. move X center w gripper 4/10")
        # self.arm.set_position_aa(axis_angle_pose=[-x_mm_gripper_width-x_mm_tuned_offset, 0, 0, 0, 0, 0], relative=True, wait=True)
        print(f"-x_mm_gripper_width-x_mm_tuned_offset: {-x_mm_gripper_width-x_mm_tuned_offset}")

        print(f" 5. move X go deeper 5/10")
        # self.arm.set_position_aa(axis_angle_pose=[-x_mm_deeper_clamp_insert, 0, 0, 0, 0, 0], relative=True, wait=True)
        print(f"-x_mm_deeper_clamp_insert: {-x_mm_deeper_clamp_insert}")

        print(f" 6. move X to recenter 6/10")
        # self.arm.set_position_aa(axis_angle_pose=[+x_mm_deeper_clamp_retract, 0, 0, 0, 0, 0], relative=True, wait=True)
        print(f"x_mm_deeper_clamp_retract: {x_mm_deeper_clamp_retract}")

        print(f" 6.5 move Y to get corn on edge of funnel 6.5/10")
        # self.arm.set_position_aa(axis_angle_pose=[0, y_mm_funnel, 0, 0, 0, 0], relative=True, wait=True)
        print(f"y_mm_funnel: {y_mm_funnel}")

    def go_to_plane(self):
        print(" ---- going to plane joint position ----")
        # self.arm.set_servo_angle(angle=[0, -45.2, -43.9, 0, 0, 0], is_radian=False, wait=True)
        self.arm.set_servo_angle(angle=[0, -78.4, -21.1, 0, 10.4, 0], is_radian=False, wait=True)

    

if __name__ == "__main__":
    print("------------------ testing main of xArm_Motion.py ------------------")