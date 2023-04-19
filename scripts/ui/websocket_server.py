#!/usr/bin/env python

import asyncio

import rospy
import websockets
from std_msgs.msg import String


async def handler(websocket):
    PUMP_pub = rospy.Publisher('PUMP_CTRL', String, queue_size=10)
    LIFT_pub = rospy.Publisher('LIFT_CTRL', String, queue_size=10)
    ARM_pub = rospy.Publisher('ARM_CTRL', String, queue=10)
    NAV_pub = rospy.Publisher('NAV_CTRL', String, queue=10)
    rospy.init_node('ws_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    STATUS = rospy.get_param('/status')
    while True:
        message = await websocket.recv()
        print("Received from Client < ", message)

        while STATUS == '1':
            print('Waiting for previous action to finish')

        if message.op == 'pump':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                message.data, message.op))
            PUMP_pub.publish(message.data)
        elif message.op == 'lift':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                message.data, message.op))
            LIFT_pub.publish(message.data)
        elif message.op == 'arm':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                message.data, message.op))
            ARM_pub.publish(message.data)
        elif message.op == 'nav':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                message.data, message.op))
            NAV_pub.publish(message.data)
            
        await websocket.send(message)
        print("Send to Client > ", message)
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
