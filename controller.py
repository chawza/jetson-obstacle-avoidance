# UI display for controlling the Robot and estabilish connection to robot's server
import asyncio
from base64 import decode
from http import client
from tracemalloc import stop
from turtle import update
import cv2
import tkinter as tk
from cv2 import exp
from websockets.client import connect
from websockets.exceptions import ConnectionClosedError
from  msvcrt import getch
import numpy as np
from threading import Thread

ACTION_DICT = {
  'w': 'FORWARD',
  's': 'BACKWARD',
  'a': 'LEFT',
  'd': 'RIGHT'
}

img = None
stop_client = asyncio.Event()
host = '192.168.100.13'

def decide_and_update_img(data):
  global img
  np_img = np.frombuffer(data, np.uint8)
  decoded_img = cv2.imdecode(np_img, -1)
  img = decoded_img.copy()

async def listen_server():
  print('Listen: Start')
  global stop_client
  while True:
    try:
      if stop_client.is_set():
        break
      conn = await connect('ws://{}:8766/cam'.format(host), max_queue=None)
      print('CAM: Server Connected')
      async for data in conn:
        decide_and_update_img(data)
        if stop_client.is_set():
          break
    except ConnectionClosedError:
      print('CAM: Disonnected')
  print('Listen: Stop')

async def app():
  global stop_client

  while not stop_client.is_set():
    try:
      print('APP: Making Connection')
      async with connect('ws://{}:8765/command'.format(host), ping_interval=1, ping_timeout=1) as ws_client:
        print('APP: Server connected')
        current_key = ''

        while True:
          key = getch()
          key = key.decode('utf-8')
          
          if key == 'f':
            await ws_client.send('SAVE_FRAME')
            continue
          elif key == 'c':
            await ws_client.send('TOGGLE_CAM_CALIBRATE')
            continue

          if current_key != key:
            current_key = key
            print('key pressed: {}'.format(current_key))
            
            if key == 'q' or key == ' ':
              await ws_client.send('QUIT')
              break

            elif key == 'r' or key == ' ':
              await ws_client.send('RESET')

            elif key == 'e':
              await ws_client.send('STOP')

            elif key == 'w' or key == 's' or key == 'a' or key == 'd':
              await ws_client.send(ACTION_DICT[key])
            
            
            await asyncio.sleep(.001)
        
        await ws_client.close()
        stop_client.set()
        print('APP: Server Disconnected')

    except ConnectionClosedError:
      await ws_client.close()
      print('APP: Reconnecting')

    print('APP: Done')

def display_image():
  print('Display Broadcast: Start')
  global img
  global stop_client
  while not stop_client.is_set():
    if img is not None:
      display_img = cv2.resize(img, dsize=(
        round(img.shape[1]*2),
        round(img.shape[0]*2),
      ))
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

  control_thread.join()
  display_img_thread.join()
  listen_server_thread.join()
  cv2.destroyAllWindows()
  exit()
  