import asyncio
import rospy
import websockets
import yaml


async def hello():
    print('type your destination')
    destination = input()
    websocket = await  websockets.connect('ws://localhost:8001/')
    test_dict = {
        'op': 'pump',
        'data': destination
    }
    yaml_message = yaml.dump(test_dict)
    print('yaml_message', yaml_message)
    await websocket.send(yaml_message)
    print("Sent To Server > {}".format(yaml_message))

if __name__ == "__main__":
    try:
        asyncio.run(hello())
    except rospy.ROSInterruptException:
        pass
  

