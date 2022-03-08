from base64 import encode
from threading import Thread
from multiprocessing import Process
from getch import getch
import cv2
from websockets.exceptions import ConnectionClosedOK
from websockets.server import WebSocketServerProtocol, serve
from Jetson import GPIO
from robot import Robot, Action as RobotAction
import asyncio
from capture_cam import StereoCams
from calibration import decode_img_to_byte, safe_frames, encode_byte_to_img
import redis

l_img = None
left_img = None
right_img = None
host='192.168.100.11'
app_port = 8765
broadcast_port = 8766
robot = Robot()
app_stop = None
cam_stop = None
r = redis.Redis('localhost', 6379, db=0)

def redis_img_sync():
  import time
  global app_stop
  global left_img
  global right_img
  global r
  while True:
    if left_img is not None:
      r.set('left_img', decode_img_to_byte(left_img))
      r.set('right_img', decode_img_to_byte(right_img))
    time.sleep(0.01)

def read_cam_task():
  global left_img
  global right_img
  camera = StereoCams()
  for left, right in camera.read(time_split=.001):
    left_img, right_img = left, right
  camera.clean_up()

async def listen_controller(websocket: WebSocketServerProtocol):
  print('Robot Server in port {}'.format(websocket.port))
  img_counter = 0
  global app_stop
  try:
    global robot
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
        print('Saving Frame')
        safe_frames(left_img, right_img, img_counter)
        img_counter += 1
      elif action == 'QUIT':
        robot.quit()
        await websocket.close()
        app_stop.set()
      else:
        print('WHAT', action)
        error_message = f'Unrecognized Action "{action}"'
        print(error_message)

  except Exception as error:
    print(error)
    robot.stop()
    app_stop.set()

async def broadcast_cam(websocket: WebSocketServerProtocol):
  print('borad cast cam in port {}'.format(websocket.port))
  r2 = redis.Redis('localhost', 6379, db=0)
  while True:
    byte_img = r2.get('left_img')
    if byte_img is not None:
      l_img = encode_byte_to_img(byte_img)
      grey_img = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
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
  try:
    print('{} starts'.format(path))
    if path == '/command':
      await listen_controller(websocket)
    if path == '/cam':
      await broadcast_cam(websocket)
  except ConnectionClosedOK:
    print('{} stops'.format(path))

  finally:
    if path == '/command':
      app_stop.set()

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

async def broadcast_server():
  print('Broadcast server Activiated')
  global cam_stop
  global broadcast_cam
  curr_lopp = asyncio.get_event_loop()
  cam_stop = asyncio.Event(loop=curr_lopp)

  async with serve(ws_handler, host, broadcast_port):
    await cam_stop.wait()

def broadcast_process_task():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(broadcast_server())

if __name__ == '__main__':
  broadcast_process = Process(target=broadcast_process_task)
  broadcast_process.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server())  
