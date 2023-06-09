#!/usr/bin/env python3

import rospy
import odrive
import math
from odrive.enums import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys

WHEEL_DIST = 0.502 #in meters
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
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.encoder.set_linear_count(0)
odrv0.axis1.encoder.set_linear_count(0)


def convert(forward,ccw):
    angular_to_linear = ccw*(WHEEL_DIST/2.0)
    right_linear_val  = (forward - angular_to_linear) * (60/TYRE_CIRCUMFERENCE) #RPM???
    left_linear_val = (forward + angular_to_linear) * (60/TYRE_CIRCUMFERENCE) #RPM???
    return left_linear_val, right_linear_val

def cmd_vel_callback(msg):
    left_linear_val, right_linear_val = convert(msg.linear.x, msg.angular.z)
    try:
        if (left_linear_val ==0) and (right_linear_val==0) and (odrv0.axis0.current_state!=1):
            odrv0.axis0.requested_state = AXIS_STATE_IDLE
            odrv0.axis1.requested_state = AXIS_STATE_IDLE
        elif odrv0.axis0.current_state!=8 and odrv0.axis1.current_state!=8:
            odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        if odrv0.axis0.current_state==8 and odrv0.axis1.current_state==8:
            odrv0.axis0.controller.input_vel = -left_linear_val/60
            odrv0.axis1.controller.input_vel = right_linear_val/60
    except:
        pass
    

    

def start_odrive():
    vel_subscribe = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=2)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            try:
            # rospy.loginfo('{}'.format(vel))
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