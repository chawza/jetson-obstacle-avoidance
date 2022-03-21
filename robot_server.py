import sys
from dotenv import load_dotenv

from depthestimation import DepthEstimator
load_dotenv()
import os
from threading import Thread
import asyncio
from getch import getch

import cv2
import numpy as np

from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from websockets.server import WebSocketServerProtocol, serve
import SharedArray as sa

from broadcast import Broadcast
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
  for left, right in camera.read(time_split=.01):
    if app_stop.is_set():
      break

    shared_left[:] = left.copy()
    shared_right[:] = right.copy()
  camera.clean_up()

async def listen_controller(websocket: WebSocketServerProtocol):
  print('Robot Server in port {}'.format(websocket.port))
  global app_stop
  global robot
  global shared_left
  global shared_right
  path = websocket.path
  img_counter = calibration.initiate_img_counter()
  if path == '/command':
    print('Robot Commmand: Connected')
    try:
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
        elif action == 'QUIT':
          robot.quit()
          await websocket.close()
          app_stop.set()
        else:
          print('WHAT', action)
          error_message = f'Unrecognized Action "{action}"'
          print(error_message)
    except ConnectionClosedError:
      print('Robot Commmand: Reconnecting')
    except ConnectionClosedOK:
      print('Robot Commmand: Disconnected')
      robot.stop()

async def broadcast_handler(websocket: WebSocketServerProtocol, _):
  depth_estimator = DepthEstimator(numDisparities=64, blockSize=9, minDisparity=6)
  path = websocket.path
  left_img = sa.attach('left')
  right_img = sa.attach('right')
  if path == '/cam':
    try:
      print('Broadcast: Connected')
      print('borad cast cam in port {}'.format(websocket.port))
      import time
      while True:
        start = time.perf_counter()
        # img_to_send = np.hstack((left_img, right_img))
        # img_to_send = cv2.cvtColor(img_to_send, cv2.COLOR_BGR2GRAY)
        # img_to_send = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        img_to_send = depth_estimator.get_depth(left_img, right_img)
        # img_to_send = depth_estimator.get_disparity(left_img, right_img)
        # img_to_send = cv2.normalize(img_to_send, None, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_64F)
        img_to_send = (img_to_send - 0) / (100 - 0)
        img_to_send = cv2.resize(img_to_send, dsize=(
          round(img_to_send.shape[1]/2),
          round(img_to_send.shape[0]/2),
        ))
        if len(img_to_send.shape) == 2:
          byte_img = calibration.encode_img_binary_to_byte(img_to_send)
        else:
          byte_img = calibration.decode_img_to_byte(img_to_send)
        await websocket.send(byte_img)
        end = time.perf_counter()
        print('time: {:.4f}'.format(round(end-start, 4)))
        await asyncio.sleep(0.1)
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

  async with serve(listen_controller, host, app_port):
    print('Robot: Server activated in {} {}'.format(host, app_port))
    await app_stop.wait()

  capture_thread.join()

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
  # initial setup
  setup_img_save_directory()
  clean_shred_memory()

  shared_left = sa.create('left', captured_img_size, np.uint8)
  shared_right = sa.create('right', captured_img_size, np.uint8)

  robot = Robot()
  camera = StereoCams()
  depth_estimator = DepthEstimator()
  
  broadcast_server = Broadcast(broadcast_handler, host=host, port=broadcast_port)
  broadcast_server.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server())

  broadcast_server.join()
  print('main ends')
