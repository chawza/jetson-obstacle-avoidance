from distutils.log import error
from threading import Thread
import time
from getch import getch
import signal
import cv2
import numpy as np
import websockets
from websockets.server import WebSocketServerProtocol, serve, unix_serve
from Jetson import GPIO
from robot import Robot, Action as RobotAction
import asyncio

left_cam_idx = 0
right_cam_idx = 1
host='192.168.100.11'
port=8765
ws_server = None
robot = Robot()
stop_robot = asyncio.Event()

# def watch_keyboard():
#   print('press "q" to quit')
#   while True:
#     time.sleep(1)
#     key = getch()
#     if key == 'q':
#       break
#   print('exit robot')
#   stop_robot.set()
#   return None

async def listen_controller(websocket: WebSocketServerProtocol):
  try:
    global robot
    async for data in websocket:
      print('Client > {}'.format(data))
      action = data
      if action == RobotAction.forward:
        robot.forward()
        await websocket.send('success')
      elif action == RobotAction.backward:
        robot.backward()
      elif action == RobotAction.left:
        robot.left()
      elif action == RobotAction.right:
        robot.right()
      elif action == RobotAction.stop:
        robot.stop()
      elif action == 'QUIT':
        robot.quit()
        stop_robot.set()
        await websocket.close()
      else:
        print('WHAT', action)
        error_message = f'Unrecognized Action "{action}"'
        await websocket.send(error_message)

  except Exception as error:
    print(error)
    robot.stop()

async def broadcast_cam(websocket: WebSocketServerProtocol):
  global robot
  global stop_robot
  cam_left = cv2.VideoCapture(left_cam_idx)
  cam_right = cv2.VideoCapture(right_cam_idx)

  if cam_left.isOpened() and cam_right.isOpened():
    print('cam opened')
    while True and not stop_robot.is_set():
      _, left_img = cam_left.read()
      _, right_img = cam_right.read()

      merged_img = np.hstack((left_img, right_img))

      buffer_img = cv2.imencode('.jpg', merged_img)[1]
      await websocket.send(buffer_img.tobytes())
      asyncio.sleep(.01)
  else:
    error_message = 'Cannot open Camera'
    await websocket.send(error_message)
    robot.quit()

  cam_left.release()
  cam_right.release()


async def ws_handler(websocket: WebSocketServerProtocol, path):
  print('Client make connection from in {}'.format(websocket.path))
  if websocket.path == '/':
    await listen_controller(websocket)
  if websocket.path == '/cam':
    await broadcast_cam(websocket)

async def initiate_server():
  global ws_server
  global stop_robot
   
  async with serve(ws_handler, host, port) as server:
    ws_server = server
    await stop_robot.wait()

if __name__ == '__main__':
  loop = asyncio.get_event_loop()
  loop.run_until_complete(initiate_server())  
  loop.close()

  if not robot.is_cleaned_up:
    robot.quit()
