#!/usr/bin/env python3

from dynamixel_sdk_examples.srv import *
import rospy

home_pose = [3192, 1926, -1, 1403, 1000, -1]
rest_pose = [3192 , 1600, -1, 800, 1000, -1]
refill_pose = [3192 , 2200, -1, 1159, 1470, -1]
serve_pose = [2081 , 2300, -1, 1317, 1470, -1]

poses = {
    "home": home_pose,
    "rest": rest_pose,
    "refill": refill_pose,
    "serve": serve_pose,
}

def set_arm_position_client(pose):
    print('Set Arm Position Client')
    rospy.wait_for_service('set_arm_position')
    try:
        set_arm_position = rospy.ServiceProxy('set_arm_position', SetArmPosition)
        response = set_arm_position(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])
        print("Result:",response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    set_arm_position_client(poses["rest"])

