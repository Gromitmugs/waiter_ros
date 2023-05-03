#!/usr/bin/env python

import asyncio

import rospy
import websockets
import yaml
from std_msgs.msg import String

FREE = 0
BUSY = 1

rospy.init_node('ws_talker', anonymous=True)

PUMP_pub = rospy.Publisher('PUMP_CTRL', String, queue_size=10)
LIFT_pub = rospy.Publisher('LIFT_CTRL', String, queue_size=10)
ARM_pub = rospy.Publisher('ARM_CTRL', String, queue_size=10)
NAV_pub = rospy.Publisher('NAV_CTRL', String, queue_size=10)


async def handler(websocket, path):
    global FREE
    global BUSY    

    yaml_message = await websocket.recv()
    print(yaml_message)

    STATUS = rospy.get_param('/status',"FREE")
    STATUS = 'FREE'

    print("Received from Client < ", yaml_message)
    dict_message = yaml.load(yaml_message)
    print(dict_message)

    if STATUS == "BUSY":
        return

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

    await websocket.send("done")



def main():
    print('Initializing Websocker Server')

    rospy.init_node('ws_talker', anonymous=True)
 
    serve = websockets.serve(handler, "", 8001)
    asyncio.get_event_loop().run_until_complete(serve)
    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    main()

