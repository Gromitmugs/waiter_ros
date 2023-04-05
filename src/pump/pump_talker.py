#!/usr/bin/env python
    # license removed for brevity
import asyncio

import rospy
import websockets
from std_msgs.msg import String


async def talker():
    websocket = await websockets.connect('ws://localhost:8765/')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        response = await websocket.recv()
        pub.publish(response)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass