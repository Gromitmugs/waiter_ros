#!/usr/bin/env python

import asyncio

import rospy
import websockets
from std_msgs.msg import String


async def handler(websocket):
    pub = rospy.Publisher('cmd_lift', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while True:
        message = await websocket.recv()
        print("Received from Client < ",message)
        print('Publishing Msg From ROS')
        pub.publish(message)
        await websocket.send(message)
        print("Send to Client > ",message)
        rate.sleep()

async def main():
    print('Initializing Websocker Server')
    async with websockets.serve(handler, "", 8001):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except rospy.ROSInterruptException:
        pass
  