import asyncio
import rospy
import websockets

async def hello():
    websocket = await  websockets.connect('ws://localhost:8001/')
    while True:
        message = input("Send a command to server? ")
        await websocket.send(message)
        print("Sent To Server > {}".format(message))
        response = await websocket.recv()
        print("Received From Server < {}".format(response))

if __name__ == "__main__":
    try:
        asyncio.run(hello())
    except rospy.ROSInterruptException:
        pass
  

