import asyncio
import rospy
import websockets
import yaml
import time


ws_client = websockets.connect('ws://localhost:8001/')
num = 0
async def hello():
    async with ws_client as websocket:
        global num
        num += 1
        test_dict = {
            'op': 'gg',
            'data': 'gay'+str(num)
        }
        yaml_message = yaml.dump(test_dict)
        print('yaml_message', yaml_message)
        await websocket.send(yaml_message)
        print("Sent To Server > {}".format(yaml_message))


while True:
    asyncio.get_event_loop().run_until_complete(hello())
    time.sleep(1)