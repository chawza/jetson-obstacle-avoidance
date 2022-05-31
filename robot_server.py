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
SBM_PRESET_FILE = os.getenv('SBM_PRESET_FILE')

calibration_path_dir = './calibration_preset'

app_stop = None
cam_stop = None
crop_radius = 20
max_n_box = round(320 / crop_radius * 2)
captured_img_size = (480, 640, 3)
min_object_distance = 2000

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

def obstacle_avoidance(disparity):
  global depth_estimator
  global crop_radius
  img_height = disparity.shape[0]
  img_width = disparity.shape[1]

  y_offset = 50
  middle_y, middle_x = round(img_height / 2), round(img_width / 2)
  middle_y += y_offset

  left_box_index = 0
  right_box_index = 0
  # detect the object distance in right side of image
  for middle_x_right in range(middle_x, img_width, crop_radius * 2):
    cropped_disparity = disparity[middle_y - crop_radius: middle_y + crop_radius, middle_x_right: middle_x_right + crop_radius * 2]
    depth_map = depth_estimator.predict_depth(cropped_disparity)
    min_distance = np.min(depth_map)

    if min_distance < min_object_distance:
      right_box_index += 1
    else:
      break

  # detect the object distance in left side of image
  for middle_x_left in range(round(img_width / 2), 0,  -(crop_radius * 2)):
    cropped_disparity = disparity[middle_y - crop_radius: middle_y + crop_radius, middle_x_left - (crop_radius * 2): middle_x_left]
    depth_map = depth_estimator.predict_depth(cropped_disparity)
    min_distance = np.min(depth_map)

    if min_distance < min_object_distance:
      left_box_index -= 1
    else:
      break
  
  box_diff = left_box_index + right_box_index
  if box_diff < 0:
    # print('GO RIGHT')
    print(f'RIGHT\t: {left_box_index / max_n_box} {right_box_index / max_n_box}')
  elif box_diff > 0:
    # print('GO LEFT')
    print(f'LEFT\t: {left_box_index / max_n_box} {right_box_index / max_n_box}')
  elif abs(left_box_index) > right_box_index:
    print("ROTATE LEFT")
  elif abs(left_box_index) < right_box_index:
    print("ROTATE RIGHT")
  else:
    print('FORWARD')

  return left_box_index, right_box_index

def draw_obstacle_box(disparity_cmap, left_box_index, right_box_index):
  global crop_radius
  img_height = disparity_cmap.shape[0]
  img_width = disparity_cmap.shape[1]

  y_offset = 50
  middle_y, middle_x = round(img_height / 2), round(img_width / 2)
  middle_y += y_offset

  for left_index_x in range(left_box_index, 0, 1):
    top_left_x = middle_x - (abs(left_index_x) * crop_radius * 2)
    bottom_right_x = middle_x  - ((abs(left_index_x) - 1) * crop_radius * 2)
    top_left = (
      (top_left_x),
      (middle_y - crop_radius)
    )
    bottom_right = (
      (bottom_right_x),
      (middle_y + crop_radius)
    )
    disparity_cmap = cv2.rectangle(disparity_cmap, top_left, bottom_right, (255, 255, 255), 3)

  for right_index_x in range(0, right_box_index + 1, 1):
    top_left_x = middle_x + (abs(right_index_x) * crop_radius * 2)
    bottom_right_x = middle_x  + ((abs(right_index_x) + 1) * crop_radius * 2)
    top_left = (
      (top_left_x),
      (middle_y - crop_radius)
    )
    bottom_right = (
      (bottom_right_x),
      (middle_y + crop_radius)
    )
    disparity_cmap = cv2.rectangle(disparity_cmap, top_left, bottom_right, (255, 255, 255), 3)
  return disparity_cmap

async def broadcast_handler(websocket: WebSocketServerProtocol, _):
  global depth_estimator
  path = websocket.path
  left_img = sa.attach('left')
  right_img = sa.attach('right')
  if path == '/cam':
    try:
      print('Broadcast: Connected')
      print('broadcast cam in port {}'.format(websocket.port))
      while True:
        ### Display RAW
        # img_to_send = np.hstack((left_img, right_img))

        ### Calibrate cams ###
        cal_left, cal_right = calibration.calibrate_imgs(left_img, right_img, preset=cam_preset)
        # cal_img_to_send = np.hstack((cal_left, cal_right))
        # # img_to_send = np.vstack((img_to_send, cal_img_to_send))
        # img_to_send = cal_img_to_send

        left_gray = cv2.cvtColor(cal_left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(cal_right, cv2.COLOR_BGR2GRAY)
        
        ### Test stereo correspondence ###
        # disparity = depth_estimator.get_disparity(left_gray, right_gray)
        # img_to_send = depth_estimator.disparity_to_colormap(disparity)
        
        ### TEST 1: Disparity and Cal left and right
        # img_to_send = np.hstack((cal_left, img_to_send, cal_right))

        ### TEST 2: Uncalibrated and Calibrated Img
        # cal_img = np.hstack((left_img, right_img))
        # uncal_img = np.hstack((cal_left, cal_right))
        # img_to_send = np.vstack((cal_img, uncal_img))

        ### TEST 3: Disparity image and Depth image
        # disparity = depth_estimator.get_disparity(left_gray, right_gray)
        # depth_map = depth_estimator.predict_depth(disparity)
        # depth_map = depth_estimator.depth_to_colormap(depth_map)
        # # # img_to_send = np.hstack((cal_left, img_to_send, depth_map))
        # img_to_send = depth_map
        
        ### TEST 4: Obstacle Avoidance and Draw Box
        disparity = depth_estimator.get_disparity(left_gray, right_gray)
        disparity_cmap = depth_estimator.disparity_to_colormap(disparity)
        left_box_index, right_box_index = obstacle_avoidance(disparity)

        if left_box_index != 0 and right_box_index != 0:
          img_to_send = draw_obstacle_box(disparity_cmap, left_box_index, right_box_index)
        else:
          img_to_send = disparity_cmap
        img_to_send = np.hstack((cal_left, img_to_send))
        
        img_to_send = cv2.resize(img_to_send, dsize=(
          round(img_to_send.shape[1]/2),
          round(img_to_send.shape[0]/2),
        ))
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
    depth_estimator.load_preset(SBM_PRESET_FILE)
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
