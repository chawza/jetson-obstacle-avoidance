from dotenv import load_dotenv
load_dotenv()

import os
from threading import Thread
import asyncio
import time
import cv2
import numpy as np
import SharedArray as sa

from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from websockets.server import WebSocketServerProtocol, serve

from datetime import datetime

from broadcast import Broadcast
from robot import Robot, Action as RobotAction
from autonomous_robot import AutonomousRobot
from depthestimation import DepthEstimator
from capture_cam import StereoCams
import calibration
import presetloader

HOST = os.getenv('APP_HOST')
APP_PORT = os.getenv('APP_PORT')
BROADCAST_PORT = os.getenv('BROADCAST_PORT')
SBM_PRESET_FILE = os.getenv('SBM_PRESET_FILE')

calibration_path_dir = './calibration_preset'

app_stop = None
cam_stop = None
crop_radius = 20
max_n_box = round(480 / (crop_radius * 2))
captured_img_size = (480, 640, 3)
disparity_img_size = (480, 640)

def read_cam_task():
  global robot_event
  global camera
  global shared_left
  global shared_right

  print('read cam task: start')
  for left, right in camera.read(time_split=.01):
    if robot_event[0] == 1:
      break
    shared_left[:] = left.copy()
    shared_right[:] = right.copy()
  camera.clean_up()
  print('read cam task: ends')

depth_map = [

]

def save_depth_map(filename='disparity_list.npy'):
  print('saving disparity list')
  presetloader.save_depth_map(depth_map, filename)
  print('disparity list saved as ', filename)

def capture_depth_map():
  global depth_estimator
  left_img = sa.attach('left')
  right_img = sa.attach('right')

  gray_left, gray_right = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
  cal_left, cal_right = calibration.calibrate_imgs(gray_left, gray_right, preset=cam_preset)

  disparity = depth_estimator.get_disparity(cal_left, cal_right)
  middle_idx = len(disparity) // 2
  
  save_disparity = np.max(disparity[middle_idx])
  x = len(depth_map) + 1
  print(f'[{x}] disparity\t: {save_disparity}')
  depth_map.append(np.float32(save_disparity))

async def listen_controller(websocket: WebSocketServerProtocol):
  global app_stop
  global shared_left
  global shared_right
  global robot_event
  global robot_process
  path = websocket.path
  if path == '/command':
    calibrate_session = calibration.CalibrateSession()
    print('Robot Commmand: Connected')
    print('Cam mode\t: ', robot_event[1])
    try:
      async for data in websocket:
        action = data
        # if action == RobotAction.forward:
        #   robot.forward()
        # elif action == RobotAction.backward:
        #   robot.backward()
        # elif action == RobotAction.left:
        #   robot.left()
        # elif action == RobotAction.right:
        #   robot.right()
        # elif action == RobotAction.stop:
        #   robot.stop()
        # elif action == RobotAction.rotate_left:
        #   robot.rotate_left()
        # elif action == RobotAction.rotate_right:
        #   robot.rotate_right()

        if action == 'INCREMENT_READ_MODE':
          robot_event[1] += 1
          if robot_event[1] == 4:
            robot_event[1] = 0
          print('Cam now\t: ', robot_event[1])

        elif action == 'SAVE_FRAME':
          calibrate_session.capture_frame(shared_left, shared_right)

        elif action == 'QUIT':
          robot_process.stop()
          app_stop.set()
        
        elif action == 'MOVE_SIGNAL':
          if robot_event[2] == 0:
            robot_event[2] = 1
          else:
            robot_event[2] = 0

        elif action == 'CAPTURE_DISPARITY_MAP':
          capture_depth_map()

        elif action == 'SAVE_DEPTH_MAP':
          save_depth_map()

        else:
          error_message = f'Unrecognized Action "{action}"'
          print(error_message)
    except ConnectionClosedError:
      print('Robot Commmand: Reconnecting')
    except ConnectionClosedOK:
      print('Robot Commmand: Disconnected')

async def broadcast_handler(websocket: WebSocketServerProtocol, _):
  global depth_estimator
  global disparity_img
  global robot_event
  global robot_process
  path = websocket.path
  left_img = sa.attach('left')
  right_img = sa.attach('right')
  sa_distance_list = sa.attach('distance_list')
  sa_disparity_img = sa.attach('disparity_img')

  if path == '/cam':
    try:
      print('Broadcast: Connected')
      print('broadcast cam in port {}'.format(websocket.port))
      while True:
        mode = robot_event[1]
        if mode == IMAGE_BRAODCAST_IDX.NORMAL:
          img_to_send = np.hstack((left_img, right_img))
          img_to_send = cv2.cvtColor(img_to_send, cv2.COLOR_BGR2GRAY)
        
        elif mode == IMAGE_BRAODCAST_IDX.CALIBRATED_IMG:
          cal_left, cal_right = calibration.calibrate_imgs(left_img, right_img, preset=cam_preset)
          img_to_send = np.hstack((cal_left, cal_right))
          img_to_send = cv2.cvtColor(img_to_send, cv2.COLOR_BGR2GRAY)

        elif mode == IMAGE_BRAODCAST_IDX.DISPARITY_IMG:
          gray_left, gray_right = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
          cal_left, cal_right = calibration.calibrate_imgs(gray_left, gray_right, preset=cam_preset)

          disparity = depth_estimator.get_disparity(cal_left, cal_right)
          img_to_send = depth_estimator.disparity_to_colormap(disparity)

        elif mode == IMAGE_BRAODCAST_IDX.DRAW_OBSTACLE:
          gray_left, gray_right = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY), cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
          cal_left, cal_right = calibration.calibrate_imgs(gray_left, gray_right, preset=cam_preset)

          disparity = depth_estimator.get_disparity(cal_left, cal_right)
          distance_list, max_disparity_list = robot_process.detect_obstacle(disparity=disparity)

          # print middle object distance
          middle_idx = len(distance_list) // 2
          print(f'idx\t: {middle_idx}\tdistance\t: {distance_list[middle_idx]}\tdisparity\t: {max_disparity_list[middle_idx]}')

          # img_to_send = depth_estimator.disparity_to_colormap(disparity)
          img_to_send = left_img.copy()
          img_to_send = AutonomousRobot.draw_obstacle_box(img_to_send, distance_list)

        ### SEND IMAGE
        # img_to_send = cv2.resize(img_to_send, dsize=(
        #   round(img_to_send.shape[1]/3),
        #   round(img_to_send.shape[0]/3),
        # ))
        byte_img = calibration.decode_img_to_byte(img_to_send)
        await websocket.send(byte_img)
        await asyncio.sleep(.1)
    except ConnectionClosedError:
      print('Broadcast: Reconnecting')
    except ConnectionClosedOK:
      print('Broadcast: Disconnected')
  else:
    print(f'unknown path {path}')


async def app_server(robot_process):
  global app_stop
  curr_lopp = asyncio.get_event_loop()
  app_stop = asyncio.Event(loop=curr_lopp)

  robot_process.start()

  async with serve(listen_controller, HOST, APP_PORT):
    print('Robot: Server activated in {} {}'.format(HOST, APP_PORT))
    await app_stop.wait()
  
  robot_event[0] = True
  robot_process.stop()
  robot_process.join()

def clean_shared_memory():
  shared_objects = sa.list()
  if len(shared_objects) < 1:
    return

  # clean read image
  delete_names = ['left', 'right', 'disparity_img', 'distance_list']
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

class IMAGE_BRAODCAST_IDX:
  NORMAL = 0
  CALIBRATED_IMG = 1
  DISPARITY_IMG = 2
  DRAW_OBSTACLE = 3

if __name__ == '__main__':
  # setup shared array
  calibration.setup_img_save_directory()
  clean_shared_memory()
  shared_left = sa.create('left', captured_img_size, np.uint8)
  shared_right = sa.create('right', captured_img_size, np.uint8)
  robot_event = sa.create('robot_event', 3, dtype=np.int8)
  disparity_img = sa.create('disparity_img', disparity_img_size, np.float32)
  distance_list = sa.create('distance_list', 16, np.float32)
  distance_list[:] = [0 for x in range(16)]

  robot_event[0] = 0 # 0 = start / 1 = stop
  robot_event[1] = IMAGE_BRAODCAST_IDX.NORMAL
  robot_event[2] = 0 # 0 = Stop / 1 = MOVE

  camera = StereoCams(1,0)
  try:
    cam_preset = presetloader.load_calibrate_map_preset()
    depth_estimator = DepthEstimator()
    depth_estimator.load_preset(SBM_PRESET_FILE)
    depth_estimator.load_depth_model()
    print(depth_estimator.get_all_sbm_properties())
  except Exception as err:
    print('unable to load cam preset')
    print(err)

  robot_process = AutonomousRobot(cam_preset, depth_estimator)

  capture_thread = Thread(target=read_cam_task)
  capture_thread.start()

  broadcast_server = Broadcast(broadcast_handler, host=HOST, port=BROADCAST_PORT, stop_event_name='robot_event')
  broadcast_server.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server(robot_process))

  broadcast_server.join()
  capture_thread.join()

  camera.clean_up()
  print('main ends')
