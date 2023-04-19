#!/usr/bin/env python

import asyncio

import rospy
import websockets
import yaml
from std_msgs.msg import String

FREE = 0
BUSY = 1

async def handler(websocket):
    global FREE
    global BUSY

    
    # PUMP_pub = rospy.Publisher('PUMP_CTRL', String, queue_size=10)
    LIFT_pub = rospy.Publisher('LIFT_CTRL', String, queue_size=10)
    # ARM_pub = rospy.Publisher('ARM_CTRL', String, queue_size=10)
    # NAV_pub = rospy.Publisher('NAV_CTRL', String, queue_size=10)
    rospy.init_node('ws_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    # STATUS = rospy.get_param('/status')
    STATUS = 'FREE'
    while True:
        yaml_message = await websocket.recv()
        print("Received from Client < ", yaml_message)
        dict_message = yaml.load(yaml_message)
        print(dict_message)
        while STATUS == BUSY:
            print('Waiting for previous action to finish')

        if dict_message['op'] == 'pump':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                dict_message['data'], dict_message['op']))
            PUMP_pub.publish(dict_message['data'])
        elif dict_message['op']  == 'lift':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                dict_message['data'], dict_message['op']))
            LIFT_pub.publish(dict_message['data'])
        elif dict_message['op'] == 'arm':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                dict_message['data'], dict_message['op']))
            ARM_pub.publish(dict_message['data'])
        elif dict_message['op'] == 'nav':
            print('Publishing {0} from websocket to ros topic according to: {1}'.format(
                dict_message['data'], dict_message['op']))
            NAV_pub.publish(dict_message['data'])
            
        await websocket.send(dict_message)
        print("Send to Client > ", dict_message)
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
