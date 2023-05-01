#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import RPi.GPIO as gpio
from time import sleep

PUMP_0 = 23
PUMP_1 = 24
PUMP_2 = 25
duration = 3

def callback(data):
    global duration
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print('Data Received', data.data)
    runPump(data.data, duration)

def runPump(pump, duration):
    print('pump received',pump)
    if pump=='0':
        gpio.output(PUMP_0, gpio.HIGH)
    elif pump=='1':
        gpio.output(PUMP_1, gpio.HIGH)
    elif pump=='2':
        gpio.output(PUMP_0, gpio.HIGH)
    sleep(duration)
    gpio.output(23, gpio.LOW)
    gpio.output(24, gpio.LOW)
    gpio.output(25, gpio.LOW)

    
def pumpControl():
    rospy.init_node('pump_controller', anonymous=True)
    rospy.Subscriber("PUMP_CTRL", String, callback)
    rospy.spin()

def init():
    global PUMP_1
    global PUMP_2
    global PUMP_3
    gpio.setmode(gpio.BCM)
    gpio.setup(PUMP_0, gpio.OUT)
    gpio.setup(PUMP_1, gpio.OUT)
    gpio.setup(PUMP_2, gpio.OUT)
    gpio.output(PUMP_0, gpio.LOW)
    gpio.output(PUMP_1, gpio.LOW)
    gpio.output(PUMP_2, gpio.LOW)

if __name__ == '__main__':
    init()
    pumpControl()