from threading import Thread, Event
from multiprocessing import Process
from getch import getch
import cv2
import websockets
from websockets.exceptions import ConnectionClosedOK
from websockets.server import WebSocketServerProtocol, serve
from Jetson import GPIO
from robot import Robot, Action as RobotAction
import asyncio
from capture_cam import StereoCams
from calibration_server import decode_img_to_byte

left_img = None
right_img = None
host='192.168.100.11'
# port=8765
robot = Robot()
app_stop = None
cam_stop = None

def read_cam_task():
  global left_img
  global right_img
  camera = StereoCams()
  for left, right in camera.read(time_split=.001):
    left_img, right_img = left, right
  camera.clean_up()

async def listen_controller(websocket: WebSocketServerProtocol):
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
  global left_img
  while True:
    if left_img is not None:
      grey_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
      grey_img = cv2.resize(grey_img, dsize=(
        round(grey_img.shape[1]/3),
        round(grey_img.shape[0]/3),
      ))
      byte_img = decode_img_to_byte(grey_img)
      await websocket.send(byte_img)
    asyncio.sleep(0.01)


async def ws_handler(websocket: WebSocketServerProtocol, _):
  path = websocket.path
  global app_stop
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
  global app_stop
  curr_lopp = asyncio.get_event_loop()
  app_stop = asyncio.Event(loop=curr_lopp)

  async with serve(ws_handler, host, 8765):
    await app_stop.wait()

async def cam_server():
  print('cam server')
  global cam_stop
  curr_lopp = asyncio.get_event_loop()
  cam_stop = asyncio.Event(loop=curr_lopp)

  async with serve(ws_handler, host, 8766):
    await cam_stop.wait()

def cam_process_task():
  capture_thread = Thread(target=read_cam_task)
  capture_thread.start()
  loop = asyncio.new_event_loop()
  loop.run_until_complete(cam_server())
  capture_thread.join()

if __name__ == '__main__':

  cam_process = Process(target=cam_process_task)
  cam_process.start()

  loop = asyncio.get_event_loop()
  loop.run_until_complete(app_server())  
  

  if not robot.is_cleaned_up:
    robot.quit()
