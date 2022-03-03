import signal
import cv2
import websockets
from websockets.server import WebSocketServerProtocol, serve, unix_serve
from Jetson import GPIO
from robot import Robot, Action as RobotAction
import asyncio

host='192.168.100.11'
port=8765
ws_server = None
robot = Robot()

async def ws_handler(websocket: WebSocketServerProtocol, path):
  print('PATH: {}'.format(path))
  global robot
  print(f'Client make connection from {websocket.host} in port {websocket.port}')
  async for data in websocket:
    print('Client > {}'.format(data))
    action = data
    if action == RobotAction.forward:
      robot.forward()
    elif action == RobotAction.backward:
      robot.backward()
    elif action == RobotAction.stop:
      robot.stop()
    elif action == 'QUIT':
      robot.quit()
      await websocket.close()
    else:
      error_message = f'Unrecognized Action "{action}"'
      await websocket.send('message:{}'.format(error_message))

async def initiate_server():
  global ws_server
  
  async with serve(ws_handler, host, port) as server:
    ws_server = server
    print('serving server in')
    await asyncio.Future()


if __name__ == '__main__':
  loop = asyncio.get_event_loop()
  loop.run_until_complete(initiate_server())
  loop.close()


