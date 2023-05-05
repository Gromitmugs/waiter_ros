#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
import RPi.GPIO as gpio
from time import sleep

DIR = 14  # Direction Pin
STEP = 15  # Step Pin
EN = 18  # Enable Pin

LM = 12 # Limit Switch

level_1_height = 24

def callback(data):
    global DIR
    global STEP
    global EN
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print('Data Received', data.data)
    moveLift(data.data)


def liftControl():
    # init subscriber node to listen to websocket
    rospy.init_node('lift_controller', anonymous=True)
    rospy.Subscriber("LIFT_CTRL", String, callback)
    rospy.spin()


def moveLift(level):
    global DIR
    global STEP
    global EN
    global LM
    global level_1_height
    print(level)
    if level == "LEVEL_1":
        if gpio.input(LM):
            print('lift is not home, moving to home')
            gpio.output(DIR, gpio.HIGH)
            while gpio.input(LM):
                gpio.output(STEP, gpio.HIGH)
                sleep(.0005)
                gpio.output(STEP, gpio.LOW)
                sleep(.0005)
            print('arrived at home, going to level 1')
        if not gpio.input(LM):
            print('lift is already home, moving to level 1')
            gpio.output(DIR, gpio.LOW)
            for i in range(level_1_height):
                revConversion()
            print('arrived at level 1')
    elif level == "HOME":
        if not gpio.input(LM):
            print('lift is already home')
            return
        print('moving lift home')
        gpio.output(DIR, gpio.HIGH)
        while gpio.input(LM):
            gpio.output(STEP, gpio.HIGH)
            sleep(.0005)
            gpio.output(STEP, gpio.LOW)
            sleep(.0005)
        print('arrived home')
    else:
        print('data unknown, skipping')
       

def revConversion():
    global DIR
    global STEP
    global EN
    for i in range(1600):
        gpio.output(STEP, gpio.HIGH)
        sleep(.0005)
        gpio.output(STEP, gpio.LOW)
        sleep(.0005)

def init():
    global DIR
    global STEP
    global EN
    global LM
    # GPIO Setup
    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    gpio.setup(LM, gpio.IN, pull_up_down=gpio.PUD_DOWN)
    gpio.setup(DIR, gpio.OUT)
    gpio.setup(STEP, gpio.OUT)
    gpio.setup(EN, gpio.OUT)
    gpio.output(EN, gpio.LOW)

if __name__ == '__main__':
    try:
        init()
        liftControl()
    except rospy.ROSInterruptException:
        pass
