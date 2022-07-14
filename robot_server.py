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
max_n_box = round(480 / crop_radius * 2)
captured_img_size = (480, 640, 3)
object_detect_distance = 2000
closest_object_distance = 400

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

def detect_obstacle(disparity):
  global depth_estimator
  global crop_radius
  img_height = disparity.shape[0]
  img_width = disparity.shape[1]

  y_offset = 50 # move scanline down relative to center
  middle_y, _ = round(img_height / 2), round(img_width / 2)
  middle_y += y_offset

  min_scan_value = []
  for x_top_left in range(0, img_width, crop_radius * 2):
    cropped_disparity = disparity[middle_y - crop_radius: middle_y + crop_radius, x_top_left: x_top_left + crop_radius * 2]
    depth_map = depth_estimator.predict_depth(cropped_disparity)
    min_distance = np.min(depth_map)
    min_scan_value.append(min_distance)

  return min_scan_value

class RgbColor:
  WHITE = (255, 255, 255)
  RED = (255, 0, 0)

def draw_obstacle_box(disparity_cmap, min_distance_list):
  global crop_radius
  img_height = disparity_cmap.shape[0]
  img_width = disparity_cmap.shape[1]

  y_offset = 50
  middle_y, _ = round(img_height / 2), round(img_width / 2)
  middle_y += y_offset

  for index, min_scan_value in enumerate(min_distance_list):
    top_left_x = index * crop_radius * 2
    bottom_right_x = (index * crop_radius * 2) + (crop_radius * 2)
    top_left = (
      (top_left_x),
      (middle_y - crop_radius)
    )
    bottom_right = (
      (bottom_right_x),
      (middle_y + crop_radius)
    )

    if min_scan_value < closest_object_distance:
      disparity_cmap = cv2.rectangle(disparity_cmap, top_left, bottom_right, RgbColor.RED, 3)
    elif min_scan_value < object_detect_distance:
      disparity_cmap = cv2.rectangle(disparity_cmap, top_left, bottom_right, RgbColor.WHITE, 3)

  return disparity_cmap

async def obstacle_avoidance(min_distance_list):
  global robot
  bot_speed = 40
  min_distance_list = np.array(min_distance_list)
  n_box = len(min_distance_list)
  scan_length = n_box / 2

  middle_depth = min_distance_list[round(len(min_distance_list) / 2)]

  if min_distance_list.min() <= closest_object_distance:
    print(f'ROBOT: STOP\t{min_distance_list.min()}')
    robot.stop()
    await asyncio.sleep(1)
    return

  left_index = None
  right_index = None

  for index, object_min_dist in enumerate(min_distance_list, 1):
    if object_min_dist < object_detect_distance:
      if left_index is None:
        left_index = index
        right_index = index
      else:
        right_index = index

  if left_index is None:
    print(f'FORWARD\t{middle_depth}')
    robot.forward(set_speed=bot_speed)
    await asyncio.sleep(.5)
    return

  left_split = left_index
  right_split = n_box - right_index

  if left_split <= right_split:
    if right_index > scan_length:
      turn_dir = right_index - scan_length
      turn_dir = (turn_dir / scan_length) * 100
    else:
      turn_dir = 10
    print(f'RIGHT\t{right_index}\t{turn_dir}')
    robot.forward(set_speed=bot_speed)
    robot.right(set_turn_dir=turn_dir)
    await asyncio.sleep(.5)
  elif left_split > right_split:
    if left_index <= scan_length:
      turn_dir = scan_length - left_index + 1
      turn_dir = (turn_dir / scan_length) * 100
    else:
      turn_dir = 10
    print(f'LEFT\t{left_index}\t{turn_dir}')
    robot.forward(set_speed=bot_speed)
    robot.left(set_turn_dir=abs(turn_dir))
    await asyncio.sleep(.5)


async def broadcast_handler(websocket: WebSocketServerProtocol, _):
  global depth_estimator
  global robot
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
        # # # img_to_send = np.vstack((img_to_send, cal_img_to_send))
        # img_to_send = cal_img_to_send

        left_gray = cv2.cvtColor(cal_left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(cal_right, cv2.COLOR_BGR2GRAY)
        
        ### Test stereo correspondence ###
        disparity = depth_estimator.get_disparity(left_gray, right_gray)
        img_to_send = depth_estimator.disparity_to_colormap(disparity)
        
        ### TEST 1: Disparity and Cal left and right
        # img_to_send = np.hstack((cal_left, img_to_send, cal_right))

        ### TEST 2: Uncalibrated and Calibrated Img
        # cal_img = np.hstack((left_img, right_img))
        # uncal_img = np.hstack((cal_left, cal_right))
        # img_to_send = np.vstack((cal_img, uncal_img))

        ### TEST 3: Disparity image and Depth image
        depth_map = depth_estimator.get_depth(left_gray, right_gray)
        depth_map = depth_estimator.depth_to_grayscale(depth_map)
        img_to_send = np.hstack((cal_left, img_to_send, depth_map))
        # img_to_send = depth_map
        
        ### TEST 4: Obstacle Avoidance and Draw Box
        # disparity = depth_estimator.get_disparity(left_gray, right_gray)
        # disparity_cmap = depth_estimator.disparity_to_colormap(disparity)
        # min_distance_list = detect_obstacle(disparity)
        # # await obstacle_avoidance(min_distance_list)

        # img_to_send = draw_obstacle_box(disparity_cmap, min_distance_list)        
        # img_to_send = np.hstack((cal_left, img_to_send))

        # ### SEND IMAGE
        # img_to_send = cv2.resize(img_to_send, dsize=(
        #   round(img_to_send.shape[1]/3),
        #   round(img_to_send.shape[0]/3),
        # ))
        byte_img = calibration.decode_img_to_byte(img_to_send)
        await websocket.send(byte_img)
        # await asyncio.sleep(.01)
        robot.stop()
        # await asyncio.sleep(2)
        await asyncio.sleep(.01)
    except ConnectionClosedError:
      print('Broadcast: Reconnecting')
    except ConnectionClosedOK:
      print('Broadcast: Disconnected')
      robot.stop()
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
  
  broadcast_server = Broadcast(broadcast_handler, host=HOST, port=BROADCAST_PORT, stop_event_name='robot_event')
  broadcast_server.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server())

  broadcast_server.join()
  camera.clean_up()
  print('main ends')
