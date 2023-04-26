#!/usr/bin/env python3

import rospy
import odrive
import math
from odrive.enums import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys

WHEEL_DIST = 0.503 #in meters
TYRE_CIRCUMFERENCE = 2*math.pi*0.035

vel_pub = rospy.Publisher('/raw_vel', Float64MultiArray, queue_size=10)
pos_pub = rospy.Publisher('/raw_pos', Float64MultiArray, queue_size=10)
raw_vel = Float64MultiArray()
raw_pos = Float64MultiArray()
vel = [0.0,0.0]
pos = [0.0,0.0]

rospy.init_node('odrive')
rospy.loginfo("Connecting to odrive...")
odrv0 = odrive.find_any()
rospy.loginfo("Initialise motors...")
odrv0.axis0.controller.input_vel = 0
odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis1.controller.input_vel = 0
odrv0.axis1.requested_state = AXIS_STATE_IDLE    

def start_odrive():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            try:
                vel[1] = float(-1*odrv0.axis0.encoder.vel_estimate * TYRE_CIRCUMFERENCE) #vel right
                vel[0] = float(odrv0.axis1.encoder.vel_estimate * TYRE_CIRCUMFERENCE) #vel left
                pos[1] = -1*odrv0.axis0.encoder.pos_estimate_counts #pos right
                pos[0] = odrv0.axis1.encoder.pos_estimate_counts #pos left
            except:
                pass
            raw_vel.data = tuple(vel)
            raw_pos.data = tuple(pos)
            vel_pub.publish(raw_vel)
            pos_pub.publish(raw_pos)
            rate.sleep()
        except KeyboardInterrupt:
            odrive_abort()

        
        
def odrive_abort():
    odrv0.axis0.controller.input_vel = 0
    odrv0.axis0.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.controller.input_vel = 0
    odrv0.axis1.requested_state = AXIS_STATE_IDLE
    odrv0.reboot()
    quit()

if __name__ == '__main__':
    try:
        rospy.loginfo("Done...")
        start_odrive()
    except rospy.ROSInterruptException:
        pass