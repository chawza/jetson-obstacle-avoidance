from unittest import expectedFailure
import cv2
import numpy as np
from capture_cam import StereoCams
from threading import Thread
import asyncio
from websockets.server import serve, WebSocketServerProtocol
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
import os

stop_server = asyncio.Event()
host = '192.168.100.11'
port = 8765
left_img = None
right_img = None
img_counter = 0
save_img_path_left = os.path.join('./calibration_img', 'left')
save_img_path_right = os.path.join('./calibration_img', 'right')

def setup_img_save_directory():
  if 'calibration_img' not in os.listdir('.'):
    os.mkdir('calibration_img')
    os.mkdir(os.path.join('calibration_img', 'left'))
    os.mkdir(os.path.join('calibration_img', 'right'))

def decode_img_to_byte(img):
  buffer_img = cv2.imencode('.jpg', img)[1]
  return buffer_img.tobytes()

async def send_img(conn: WebSocketServerProtocol):
  global left_img
  global right_img
  camera = StereoCams()
  for left, right in camera.read(time_split=.01):
    left_img, right_img = left, right
    merged_img = np.hstack((left_img, right_img))
    merged_img = cv2.resize(merged_img, dsize=(
      round(merged_img.shape[1]/2),
      round(merged_img.shape[0]/2),
    ))
    grey_left = cv2.cvtColor(merged_img, cv2.COLOR_BGR2GRAY)
    byte_img = decode_img_to_byte(grey_left)
    await conn.send(byte_img)
  camera.clean_up()

def save_current_frame():
  global left_img
  global right_img
  global img_counter

  left_result = cv2.imwrite(
    os.path.join(save_img_path_left, 'img_{}.jpg'.format(img_counter)),
    left_img,
  )
  right_result = cv2.imwrite(
    os.path.join(save_img_path_right, 'img_{}.jpg'.format(img_counter)),
    right_img,
  )

  if left_result and right_result:
    print('img saved')
  else:
    print('Saved Failed')


async def accept_command(conn: WebSocketServerProtocol):
  global img_counter
  async for command in conn:
    command = command.lower()
    if command == 'save':
      print('SAVE')
      save_current_frame()
      img_counter += 1
      
    elif command == 'quit':
      print('QUIT')
      break

async def ws_handler(conn: WebSocketServerProtocol, _):
  global stop_server
  path = conn.path
  try:
    print('{} starts'.format(path))
    if path == '/stream_img':
      await send_img(conn)
    elif path == '/command':
      await accept_command(conn)
  except ConnectionClosedOK:
    print('{} stops'.format(path))

  finally:
    if path == '/command':
      stop_server.set()

async def start_server():
  async with serve(ws_handler, host, port):
    await stop_server.wait()


if __name__ == '__main__':
  setup_img_save_directory()
  main_loop = asyncio.get_event_loop()
  main_loop.run_until_complete(start_server())

