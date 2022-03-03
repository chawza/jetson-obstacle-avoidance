# UI display for controlling the Robot and estabilish connection to robot's server
import asyncio
from http import client
import cv2
import tkinter as tk
import websockets
from websockets.client import WebSocketClientProtocol, connect
import time
from  msvcrt import getch
import numpy as np
from threading import Thread

ACTION_DICT = {
  'w': 'FORWARD',
  's': 'BACK',
  'a': 'LEFT',
  'd': 'RIGHT'
}

img = None

async def create_connection(host='192.168.100.11', port=8765):
  print(f'connecting to {host}:{port}')
  return await connect(f'ws://{host}:{port}')


async def recieve_img(ws_client: WebSocketClientProtocol):
  print('receive img starts')
  global img
  async for data in ws_client:
    try:
      img_buffer = bytes(data)
      img = cv2.imdecode(img_buffer)
    except TypeError as Err:
      print('Server > {}'.format(data))


async def app(ws_client: WebSocketClientProtocol):
  print('entered App')
  current_key = ''
  print('Robot ready to move')

  while True and not ws_client.closed:
    if img is not None:
      cv2.imshow('img', img)

    cv2.waitKey(1)
    key = getch()
    key = key.decode('utf-8')
    
    if current_key != key:
      current_key = key
      print('key pressed: {}'.format(current_key))
      
      if key == 'q' or key == ' ':
        await ws_client.send('QUIT')
        break

      if key == 'r' or key == ' ':
        await ws_client.send('RESET')
        continue

      if key == 'w' or key == 's' or key == 'a' or key == 'd':
        await ws_client.send(ACTION_DICT[key])
        continue
  
  ws_client.close()
  print('closing app')
  return 'App Done'

async def main():
  print('main() starts')
  global ws_client
  
  ws_client = await create_connection()
  print('connection created')

  await asyncio.gather(
    recieve_img(ws_client),
    app(ws_client)
  )

if __name__ == '__main__':
  print('Robot Controller Starts')
  loop = asyncio.get_event_loop()
  loop.run_until_complete(main())
  loop.close()
  