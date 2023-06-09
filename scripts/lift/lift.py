#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import RPi.GPIO as gpio
from time import sleep

DIR = 18  # Direction Pin
STEP = 25  # Step Pin
EN = 12  # Enable Pin


# data type: "TOP" "BOTTOM"
def callback(data):
    global DIR
    global STEP
    global EN
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    moveLift(data.data)


def liftControl():
    # init subscriber node to listen to websocket
    rospy.init_node('lift_controller', anonymous=True)

    rospy.Subscriber("cmd_lift", String, callback)
    global DIR
    global STEP
    global EN
    # GPIO Setup
    gpio.setmode(gpio.BCM)
    gpio.setup(DIR, gpio.OUT)
    gpio.setup(STEP, gpio.OUT)
    gpio.setup(EN, gpio.OUT)
    gpio.output(EN, gpio.LOW)

    rospy.spin()

def stopLift():
    global EN
    gpio.output(EN, gpio.HIGH)
    print("Lift Stopped")

def startLift():
    global EN
    gpio.output(EN, gpio.LOW)
    print("Lift Started")

def moveLift(level):
    global DIR
    global STEP
    global EN
    # move lift down
    if level == "TOP":
        gpio.output(DIR, gpio.HIGH)
        revConversion(30)

    else:  # move lift down
        gpio.output(DIR, gpio.LOW)
        revConversion(30)

# length unit : millimeter
def revConversion(length):
    global DIR
    global STEP
    global EN
    # 1 rev = 10 mm
    rev = length / 10.00
    for i in range(rev*1600):
        gpio.output(STEP, gpio.HIGH)
        sleep(.0005)
        gpio.output(STEP, gpio.LOW)
        sleep(.0005)


if __name__ == '__main__':
    try:
        liftControl()
    except rospy.ROSInterruptException:
        pass
