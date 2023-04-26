#!/usr/bin/env python3

import rospy
from waiter_ros.srv import *

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
    set_arm_position_client(poses["home"])

