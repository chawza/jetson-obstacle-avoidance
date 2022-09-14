# UI display for controlling the Robot and estabilish connection to robot's server
from dotenv import load_dotenv
load_dotenv()

import os
import asyncio
from threading import Thread
from  msvcrt import getch

import cv2
import numpy as np

from websockets.client import connect
from websockets.exceptions import ConnectionClosedError, ConnectionClosedOK
import calibration
from robot import Action

host = os.getenv('APP_HOST')
app_port = os.getenv('APP_PORT')
broadcast_port = os.getenv('BROADCAST_PORT')

img = None
stop_client = asyncio.Event()

def decide_and_update_img(data):
  global img
  decoded_img = calibration.encode_byte_to_img(data)
  img = decoded_img.copy()

async def listen_server():
  print('Listen: Start')
  global stop_client
  while True:
    try:
      if stop_client.is_set():
        break
      conn = await connect('ws://{}:{}/cam'.format(host, broadcast_port), max_queue=None)
      print('CAM: Server Connected')
      async for data in conn:
        if stop_client.is_set():
          await conn.close()
        decide_and_update_img(data)
    except ConnectionClosedError:
      print('CAM: Reconnecting')
    except ConnectionClosedOK:
      print('CAM: Disonnected')
      break
    except ConnectionRefusedError:
      print('Server Lost: Quitting')
      break

  print('Listen: Stop')

ACTION_DICT = {
  'w': Action.forward,
  's': Action.backward,
  'a': Action.left,
  'd': Action.right,
  'q': Action.rotate_left,
  'e': Action.rotate_right,
  'x': Action.stop
}

async def app():
  global stop_client

  while not stop_client.is_set():
    try:
      print('APP: Making Connection {} {}'.format(host, 8765))
      async with connect('ws://{}:{}/command'.format(host, app_port), ping_interval=1, ping_timeout=1, max_queue=1) as ws_client:
        print('APP: Server connected')

        while True:
          key = getch()
          key = key.decode('utf-8')
          print('key pressed: {}'.format(key))
          
          if key in ACTION_DICT:
            await ws_client.send(ACTION_DICT[key])
          elif key == 'f':
            await ws_client.send('SAVE_FRAME')
          elif key == 'z' or key == ' ':
            await ws_client.send('QUIT')
            break
          elif key == 'c':
            await ws_client.send('INCREMENT_READ_MODE')
          elif key == 'm':
            await ws_client.send('MOVE_SIGNAL')
          elif key == 'h':
            await ws_client.send('SAVE_DEPTH_MAP')
          elif key == 'j':
            await ws_client.send('CAPTURE_DISPARITY_MAP')
        
        await ws_client.close()
        stop_client.set()
        print('APP: Client Disconnected')

    except ConnectionClosedError:
      await ws_client.close()
      print('APP: Reconnecting')

    print('APP: Done')

def display_image():
  print('Display Broadcast: Start')
  global img
  global stop_client
  while True:
    if img is not None:
      display_img = img
      # display_img = cv2.resize(img, dsize=(
      #   round(img.shape[1]*3),
      #   round(img.shape[0]*3),
      # ))
      cv2.imshow('img', display_img)
      cv2.waitKey(10)
    
    if stop_client.is_set():
      break
  print('Display Broadcast: Stop')

def listen_thread():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(listen_server())

def app_thread():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(app())

if __name__ == '__main__':
  print('Robot Controller Starts')
  control_thread = Thread(target=app_thread)
  control_thread.start()

  listen_server_thread = Thread(target=listen_thread)
  listen_server_thread.start()

  display_img_thread = Thread(target=display_image)
  display_img_thread.start()

  display_img_thread.join()
  listen_server_thread.join()
  control_thread.join()
  cv2.destroyAllWindows()
  exit()
  