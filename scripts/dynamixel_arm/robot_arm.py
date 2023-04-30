#!/usr/bin/env python3

import rospy
from waiter_ros.srv import *
from robot_arm_params import ARM_POSES
from std_msgs.msg import String

def set_arm_position_client(poses):
    rospy.wait_for_service('set_arm_position')
    try:
        set_arm_position = rospy.ServiceProxy('set_arm_position', SetArmPosition)
        for pose in poses:
            response = set_arm_position(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])
            print("Result:",response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def callback(data):
    if data.data.isnumeric():
        gripper_control_client(data.data)
    else:
        set_arm_position_client(ARM_POSES[data.data])

def gripper_control_client(operation):
    rospy.wait_for_service('gripper_control')
    try:
        gripper_control = rospy.ServiceProxy('gripper_control', GripperControl)
        response = gripper_control(operation)
        print("Result:",response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('robot_arm', anonymous=True)
    rospy.Subscriber("/arm_ctrl", String, callback)
    rospy.spin()


    

