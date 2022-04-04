from dotenv import load_dotenv
load_dotenv()

import os
from threading import Thread
import asyncio
from getch import getch
import time

import cv2
import numpy as np
import SharedArray as sa

from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from websockets.server import WebSocketServerProtocol, serve

from broadcast import Broadcast
from robot import Robot, Action as RobotAction
from depthestimation import DepthEstimator
from capture_cam import StereoCams
import calibration
import presetloader

HOST = os.getenv('APP_HOST')
APP_PORT = os.getenv('APP_PORT')
BROADCAST_PORT = os.getenv('BROADCAST_PORT')

calibration_path_dir = './calibration_preset'

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
  calibrate_session = calibration.CalibrateSession()
  if path == '/command':
    print('Robot Commmand: Connected')
    try:
      async for data in websocket:
        action = data
        if action == RobotAction.forward:
          robot.forward()
        elif action == RobotAction.backward:
          robot.backward()
        elif action == RobotAction.left:
          robot.left()
        elif action == RobotAction.right:
          robot.right()
        elif action == RobotAction.stop:
          robot.stop()
        elif action == RobotAction.rotate_left:
          robot.rotate_left()
        elif action == RobotAction.rotate_right:
          robot.rotate_right()

        elif action == 'SAVE_FRAME':
          calibrate_session.capture_frame(shared_left, shared_right)

        elif action == 'QUIT':
          robot.quit()
          app_stop.set()
          await websocket.close()
        else:
          error_message = f'Unrecognized Action "{action}"'
          print(error_message)

        robot.print_debug()
    except ConnectionClosedError:
      print('Robot Commmand: Reconnecting')
      robot.stop()
    except ConnectionClosedOK:
      print('Robot Commmand: Disconnected')
      robot.stop()

async def broadcast_handler(websocket: WebSocketServerProtocol, _):
  global depth_estimator
  path = websocket.path
  left_img = sa.attach('left')
  right_img = sa.attach('right')
  if path == '/cam':
    try:
      print('Broadcast: Connected')
      print('borad cast cam in port {}'.format(websocket.port))
      while True:
        ### Calibrate cams ###
        cal_left, cal_right = calibration.calibrate_imgs(left_img, right_img, preset=cam_preset)
        left_gray = cv2.cvtColor(cal_left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(cal_right, cv2.COLOR_BGR2GRAY)
        # cal_img_to_send = np.hstack((left_gray, right_gray))
        # img_to_send = cal_img_to_send
        
        ### Test stereo correspondence ###
        # disparity = depth_estimator.get_disparity(left_gray, right_gray)
        # img_to_send = depth_estimator.disparity_to_colormap(disparity)

        ### Test Depth Estimation
        disparity = depth_estimator.get_disparity(left_gray, right_gray)
        img_to_send = depth_estimator.disparity_to_colormap(disparity)
        
        crop_radius = 10
        middle_y, middle_x = round(left_gray.shape[0] / 2), round(left_gray.shape[1] / 2)
        img_to_send = cv2.rectangle(img_to_send, (middle_x - crop_radius, middle_y - crop_radius), (middle_x + crop_radius, middle_y + crop_radius), (255, 255, 255), 1)

        crop_disparity = disparity[middle_y - crop_radius: middle_y + crop_radius, middle_x - crop_radius: middle_x + crop_radius]
        depth_map = depth_estimator.predict_depth(crop_disparity)

        print('min', np.min(depth_map))

        ### Test depth estimation ###
        # img_to_send = depth_estimator.depth_to_colormap(img_to_send)

        # img_to_send = cv2.resize(img_to_send, dsize=(
        #   round(img_to_send.shape[1]/2),
        #   round(img_to_send.shape[0]/2),
        # ))
        byte_img = calibration.decode_img_to_byte(img_to_send)
        start = time.perf_counter()
        await websocket.send(byte_img)
        end = time.perf_counter()
        # print('time: {:.4f}'.format(end-start))
        await asyncio.sleep(.1)
    except ConnectionClosedError:
      print('Broadcast: Reconnecting')
    except ConnectionClosedOK:
      print('Broadcast: Disconnected')
  else:
    print(f'unknown path {path}')


async def app_server():
  capture_thread = Thread(target=read_cam_task)
  capture_thread.start()

  global app_stop
  curr_lopp = asyncio.get_event_loop()
  app_stop = asyncio.Event(loop=curr_lopp)

  async with serve(listen_controller, HOST, APP_PORT):
    print('Robot: Server activated in {} {}'.format(HOST, APP_PORT))
    await app_stop.wait()
    robot_event[0] = True

  capture_thread.join()

def clean_shared_memory():
  shared_objects = sa.list()
  if len(shared_objects) < 1:
    return

  # clean read image
  delete_names = ['left', 'right']
  for obj in shared_objects:
    obj_name = obj.name.decode()
    if obj_name in delete_names:
      sa.delete(obj_name)
  
  # setup robot event
  try:
    sa.attach('robot_event')
    sa.delete('robot_event')
  except FileNotFoundError:
    pass


if __name__ == '__main__':
  # setup shared array
  calibration.setup_img_save_directory()
  clean_shared_memory()
  shared_left = sa.create('left', captured_img_size, np.uint8)
  shared_right = sa.create('right', captured_img_size, np.uint8)
  robot_event = sa.create('robot_event', 2, dtype=bool)

  robot = Robot()
  camera = StereoCams(0,1)
  try:
    cam_preset = presetloader.load_calibrate_map_preset()
    depth_estimator = DepthEstimator()
    depth_estimator.load_preset()
    depth_estimator.load_depth_model()
    print(depth_estimator.get_all_sbm_properties())
  except Exception as err:
    print('unable to load cam preset')
    print(err)
  
  broadcast_server = Broadcast(broadcast_handler, host=HOST, port=BROADCAST_PORT, stop_event_name='robot_event')
  broadcast_server.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server())

  broadcast_server.join()
  camera.clean_up()
  print('main ends')
