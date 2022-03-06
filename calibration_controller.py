from msvcrt import getch
from socket import timeout
from websockets.client import connect
from threading import Thread, Event
import asyncio
import cv2
import numpy as np

img = None
host='192.168.100.11'
port=8765
stop_client = Event()

def display_img():
  global img
  while True:
    if stop_client.is_set():
      break
    if img is not None:
      cv2.imshow('robot_cam', img)
    cv2.waitKey(1)

async def stream_cam_server():
  global img
  url = 'ws://{}:{}/stream_img'.format(host, port)
  conn = await connect(url)
  async for byte_img in conn:
    np_img = np.frombuffer(byte_img, np.uint8)
    decoded_img = cv2.imdecode(np_img, -1)
    img = decoded_img.copy()

    if stop_client.is_set():
      await conn.close()

async def control_session():
  url = 'ws://{}:{}/command'.format(host, port)
  conn = await connect(url)
  
  current_key = ''

  print('Enter command')
  while True:
    key = getch()
    key = key.decode('utf-8')
    
    if current_key != key:
      current_key = key
      print('key pressed: {}'.format(current_key))
      
      if key == 's' or key == ' ':
        await conn.send('save')
        print('img saved')
      
      if key == 'q':
        await conn.send('quit')
        break

    await asyncio.sleep(.001)
  
  await conn.close()
  stop_client.set()


def stream_cam_task():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(stream_cam_server())

def command_robot_task():
  loop = asyncio.new_event_loop()
  loop.run_until_complete(control_session())

if __name__ == '__main__':
  display_thread = Thread(target=display_img)
  stream_thread = Thread(target=stream_cam_task)
  command_thread = Thread(target=command_robot_task)

  display_thread.start()
  stream_thread.start()
  command_thread.start()
  cv2.destroyAllWindows()

