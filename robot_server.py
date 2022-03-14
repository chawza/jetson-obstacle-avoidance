import sys
from dotenv import load_dotenv
load_dotenv()
import os
from threading import Thread
from multiprocessing import Process
import asyncio
from getch import getch

import cv2
import redis
import numpy as np

from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from websockets.server import WebSocketServerProtocol, serve
from robot import Robot, Action as RobotAction
from capture_cam import StereoCams
from calibration import decode_img_to_byte, safe_frames, encode_byte_to_img, setup_img_save_directory
import calibration


l_img = None
left_img = None
right_img = None
host=os.getenv('APP_HOST')
app_port = os.getenv('APP_PORT')
broadcast_port = os.getenv('BROADCAST_PORT')
app_stop = None
cam_stop = None
r = redis.Redis('localhost', 6379, db=0)

def redis_img_sync():
  import time
  global app_stop
  global left_img
  global right_img
  global r

  while app_stop is None:
    time.sleep(.1)

  while not app_stop.is_set():
    if left_img is not None:
      r.set('left_img', decode_img_to_byte(left_img))
      r.set('right_img', decode_img_to_byte(right_img))
    time.sleep(0.01)

def read_cam_task():
  global left_img
  global right_img
  global app_stop
  global camera
  for left, right in camera.read(time_split=.001):
    if app_stop.is_set():
      break
    left_img, right_img = left, right
  camera.clean_up()

async def listen_controller(websocket: WebSocketServerProtocol):
  print('Robot Server in port {}'.format(websocket.port))
  img_counter = calibration.initiate_img_counter()
  global app_stop
  global robot
  global camera
  async for data in websocket:
    action = data
    if action == RobotAction.forward:
      print('ROBOT: FORWARD')
      robot.forward()
      await websocket.send('success')
    elif action == RobotAction.backward:
      print('ROBOT: BACKWARD')
      robot.backward()
    elif action == RobotAction.left:
      print('ROBOT: LEFT')
      robot.left()
    elif action == RobotAction.right:
      print('ROBOT: RIGHT')
      robot.right()
    elif action == RobotAction.stop:
      print('ROBOT: STOP')
      robot.stop()
    elif action == 'SAVE_FRAME':
      print('Saving Frame {}'.format(img_counter))
      safe_frames(left_img, right_img, img_counter)
      img_counter += 1
    elif action == 'CALIBRATE':
      calibration.calibrate_cam()
    elif action == 'TOGGLE_CAM_CALIBRATE':
      camera.is_calibrate = not camera.is_calibrate
      print('Toggle from {} to {}'.format(not camera.is_calibrate, camera.is_calibrate))
    elif action == 'QUIT':
      robot.quit()
      await websocket.close()
      app_stop.set()
    else:
      print('WHAT', action)
      error_message = f'Unrecognized Action "{action}"'
      print(error_message)


async def broadcast_cam(websocket: WebSocketServerProtocol):
  print('borad cast cam in port {}'.format(websocket.port))
  r2 = redis.Redis('localhost', 6379, db=0)
  while True:
    byte_left = r2.get('left_img')
    byte_right = r2.get('right_img')
    if byte_left is not None:
      l_img = encode_byte_to_img(byte_left)
      r_img = encode_byte_to_img(byte_right)
      img = np.hstack((l_img, r_img))
      grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      # grey_img = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
      grey_img = cv2.resize(grey_img, dsize=(
        round(grey_img.shape[1]/3),
        round(grey_img.shape[0]/3),
      ))
      byte_img = decode_img_to_byte(grey_img)
      await websocket.send(byte_img)
    asyncio.sleep(0.1)


async def ws_handler(websocket: WebSocketServerProtocol, _):
  global app_stop
  path = websocket.path

  if path == '/command':
    try:
      print('Robot Commmand: Connected')
      await listen_controller(websocket)
    except ConnectionClosedError:
      print('Robot Command: Disconnected')
      robot.stop()
  if path == '/cam':
    try:
      print('Broadcast: Connected')
      await broadcast_cam(websocket)
    except ConnectionClosedError:
      print('Broadcast: Disconnected')

async def app_server():
  capture_thread = Thread(target=read_cam_task)
  capture_thread.start()
  redis_sync_thread = Thread(target=redis_img_sync)
  redis_sync_thread.start()

  global app_stop
  curr_lopp = asyncio.get_event_loop()
  app_stop = asyncio.Event(loop=curr_lopp)

  async with serve(ws_handler, host, app_port):
    await app_stop.wait()

  capture_thread.join()
  redis_sync_thread.join()

async def broadcast_server():
  print('Broadcast server Activiated')
  global cam_stop
  curr_lopp = asyncio.get_event_loop()
  cam_stop = asyncio.Event(loop=curr_lopp)

  async with serve(ws_handler, host, broadcast_port):
    await cam_stop.wait()

def broadcast_process_task():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(broadcast_server())

if __name__ == '__main__':
  calibrate_arg = False
  if '--calibrate-cam' in sys.argv:
    print('camera calibration activated')
    calibrate_arg = True

  robot = Robot()
  camera = StereoCams(calibrate=calibrate_arg)

  setup_img_save_directory()
  broadcast_process = Process(target=broadcast_process_task)
  broadcast_process.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server())  
