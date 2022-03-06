# UI display for controlling the Robot and estabilish connection to robot's server
import asyncio
from base64 import decode
from http import client
from turtle import update
import cv2
import tkinter as tk
from cv2 import exp
import websockets
from websockets.client import WebSocketClientProtocol, connect
import time
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

async def create_connection(host='192.168.100.11', port=8765, path='/'):
  print(f'connecting to {host}:{port}')
  return await connect(f'ws://{host}:{port}{path}')

def update_img(data):
  global img
  np_img = np.frombuffer(data, np.uint8)
  decoded_img = cv2.imdecode(np_img, -1)
  img = decoded_img.copy()

async def listen_server():
  print('CAM: Making Connection')
  conn = await connect('ws://192.168.100.11:8765/cam')
  print('CAM: Server Connected')
  async for data in conn:
    try:
      update_img(data)
    except TypeError as Err:
      # print('Server > {}'.format(data))
      pass  

async def app():
  global stop_client
  print('APP: Making Connection')
  async with connect('ws://192.168.100.11:8765/') as ws_client:
    print('APP: Server connected')
    current_key = ''

    while True:
      key = getch()
      key = key.decode('utf-8')
      
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
    
    await ws_client.close()
    stop_client.set()
    print('closing app')
    return 'App Done'

def display_image():
  global img
  global stop_client
  while True and not stop_client.is_set():
    try:
      cv2.imshow('img', img)
      cv2.waitKey(1)
    except Exception as err:
      print(err)
  cv2.destroyAllWindows()

def listen_thread():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(listen_server())

def app_thread():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(app())

if __name__ == '__main__':
  print('Robot Controller Starts')
  # listen_server_thread = Thread(target=listen_thread)
  control_thread = Thread(target=app_thread)

  while not stop_client.is_set():
    control_thread.start()

  # listen_server_thread.start()
  control_thread.start()
  display_image()
  