import asyncio
import websockets
from websockets.server import WebSocketServerProtocol
import cv2
import numpy as np
import threading

img = np.zeros((500, 500, 3))
running = True

def show_img():
  global running
  while running:
    cv2.imshow('Recieved Img', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
      running = False
  cv2.destroyAllWindows()

async def ws_handler(websocket: WebSocketServerProtocol):
  global img
  global running
  print(f'Client make connection from {websocket.host} in port {websocket.port}')
  try:
    while running:
      async for message in websocket:
        if not running:
          break
        np_img = np.frombuffer(message, np.uint8)
        decoded_img = cv2.imdecode(np_img, -1)
        img = decoded_img.copy()

  except Exception as err:
    print(err)

  finally:
    await websocket.close()
    print('Conenction closed')

async def main():
  global running
  print('Server websocket started')
  async with websockets.serve(ws_handler, '192.168.100.10', 8765):
    await asyncio.Future()


def get_images():
  asyncio.run(main())

socket_thread = threading.Thread(target=get_images)

if __name__ == '__main__':
  socket_thread.start()
  show_img()
  socket_thread.join()
  