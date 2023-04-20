#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
import RPi.GPIO as gpio
from time import sleep

PUMP0 = 24
PUMP1 = 13
PUMP2 = 15

def init():
    global PUMP0
    global PUMP1
    global PUMP2

    # GPIO Initial Setup
    gpio.setmode(gpio.BCM)
    gpio.setup(PUMP0, gpio.OUT)
    gpio.setup(PUMP1, gpio.OUT)
    gpio.setup(PUMP2, gpio.OUT)

    # PUMP Initial state: turned off
    # HIGH or LOW state depending on circuit direction
    gpio.output(PUMP0, gpio.LOW) 
    gpio.output(PUMP1, gpio.LOW)
    gpio.output(PUMP2, gpio.LOW)

# data type : pumpID : "0" "1" "2" : bev1 bev2 bev3 respectively
def callback(data):
    global PUMP0
    global PUMP1
    global PUMP2
    rospy.loginfo(rospy.get_caller_id() + ' Data Received: ' data.data)
    rospy.set_param('ROBOT_STATUS', '1') # assuming 1 is not free
    dispensePump(data.data)
    rospy.set_param('ROBOT_STATUS', '0') # assuming 0 is free


def pumpControl():
    # init subscriber node to listen to websocket
    rospy.init_node('pump_controller', anonymous=True)
    rospy.Subscriber("cmd_pump", String, callback)
    rospy.spin()


def dispensePump(pumpID):
    global PUMP0
    global PUMP1
    global PUMP2
    print(pumpID)
    
    if pumpID == "0":
        print("Dispensing Selected Beverage [id=", pumpID, "]")
        gpio.output(PUMP0, gpio.HIGH) # turn on pump0
        sleep(8) # sleep 8 sec = 240 ml (pump rate 1.8L/min)
        gpio.output(PUMP0, gpio.LOW) # turn off pump
        print("Done Filling Beverage, Enjoy")
    elif pumpID == "1":
        print("Dispensing Selected Beverage [id=", pumpID, "]")
        gpio.output(PUMP1, gpio.HIGH) # turn on pump1
        sleep(8) # sleep 8 sec = 240 ml (pump rate 1.8L/min)
        gpio.output(PUMP1, gpio.LOW) # turn off pump
        print("Done Filling Beverage, Enjoy")
    elif pumpID == "2":
        print("Dispensing Selected Beverage [id=", pumpID, "]")
        gpio.output(PUMP2, gpio.HIGH) # turn on pump2
        sleep(8) # sleep 8 sec = 240 ml (pump rate 1.8L/min)
        gpio.output(PUMP2, gpio.LOW) # turn off pump
        print("Done Filling Beverage, Enjoy")
    else:
        print("Invalid Pump ID")
        # turn off all pump
        gpio.output(PUMP0, gpio.LOW)
        gpio.output(PUMP1, gpio.LOW)
        gpio.output(PUMP2, gpio.LOW) 

if __name__ == '__main__':
    try:
        init()
        pumpControl()
    except rospy.ROSInterruptException:
        pass
