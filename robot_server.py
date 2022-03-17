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
import SharedArray as sa

from robot import Robot, Action as RobotAction
from capture_cam import StereoCams
from calibration import decode_img_to_byte, safe_frames, encode_byte_to_img, setup_img_save_directory
import calibration

host=os.getenv('APP_HOST')
app_port = os.getenv('APP_PORT')
broadcast_port = os.getenv('BROADCAST_PORT')
app_stop = None
cam_stop = None

captured_img_size = (480, 640, 3)

def read_cam_task():
  global app_stop
  global camera
  global shared_left
  global shared_right
  for left, right in camera.read(time_split=.001):
    if app_stop.is_set():
      break

    shared_left[:] = left.copy()
    shared_right[:] = right.copy()
  camera.clean_up()

async def listen_controller(websocket: WebSocketServerProtocol):
  print('Robot Server in port {}'.format(websocket.port))
  img_counter = calibration.initiate_img_counter()
  global app_stop
  global robot
  global camera
  global shared_left
  global shared_right
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
      safe_frames(shared_left, shared_right, img_counter)
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
  left_img = sa.attach('left')
  right_img = sa.attach('right')
  while True:
    img = np.hstack((left_img, right_img))
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grey_img = cv2.resize(grey_img, dsize=(
      round(grey_img.shape[1]/3),
      round(grey_img.shape[0]/3),
    ))
    byte_img = decode_img_to_byte(grey_img)
    await websocket.send(byte_img)
    asyncio.sleep(0.01)


async def ws_handler(websocket: WebSocketServerProtocol, _):
  path = websocket.path

  if path == '/command':
    try:
      print('Robot Commmand: Connected')
      await listen_controller(websocket)
    except ConnectionClosedError:
      print('Robot Commmand: Reconnecting')
    except ConnectionClosedOK:
      print('Robot Commmand: Disconnected')
      robot.stop()

async def broadcast_handler(websocket: WebSocketServerProtocol, _):
  path = websocket.path

  if path == '/cam':
    try:
      print('Broadcast: Connected')
      await broadcast_cam(websocket)
    except ConnectionClosedError:
      print('Broadcast: Reconnecting')
    except ConnectionClosedOK:
      print('Broadcast: Disconnected')

async def app_server():
  capture_thread = Thread(target=read_cam_task)
  capture_thread.start()

  global app_stop
  curr_lopp = asyncio.get_event_loop()
  app_stop = asyncio.Event(loop=curr_lopp)

  async with serve(ws_handler, host, app_port):
    await app_stop.wait()

  capture_thread.join()

async def broadcast_server():
  print('Broadcast server Activiated')
  global cam_stop
  curr_lopp = asyncio.get_event_loop()
  cam_stop = asyncio.Event(loop=curr_lopp)

  async with serve(broadcast_handler, host, broadcast_port):
    await cam_stop.wait()

def broadcast_process_task():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(broadcast_server())

def clean_shred_memory():
  shared_objects = sa.list()
  if len(shared_objects) < 1:
    return

  delete_names = ['left', 'right']
  for obj in shared_objects:
    obj_name = obj.name.decode()
    if obj_name in delete_names:
      sa.delete(obj_name)

if __name__ == '__main__':
  calibrate_arg = False
  if '--calibrate-cam' in sys.argv:
    print('camera calibration activated')
    calibrate_arg = True

  clean_shred_memory()
  shared_left = sa.create('left', captured_img_size, np.uint8)
  shared_right = sa.create('right', captured_img_size, np.uint8)

  robot = Robot()
  camera = StereoCams(calibrate=calibrate_arg)

  setup_img_save_directory()
  broadcast_process = Process(target=broadcast_process_task)
  broadcast_process.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server())
  print('main ends')
