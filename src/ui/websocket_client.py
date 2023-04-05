import asyncio

import websockets


@asyncio.coroutine
async def hello():
    websocket = await websockets.connect('ws://localhost:8765/')
    name = input("What's your name? ")
    await websocket.send(name)
    print("> {}".format(name))
    greeting = await websocket.recv()
    print("< {}".format(greeting))

asyncio.get_event_loop().run_until_complete(hello())