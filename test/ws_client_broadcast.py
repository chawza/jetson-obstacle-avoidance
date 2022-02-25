import asyncio
import numpy as np
import websockets
import time
import cv2

async def main():
  async with websockets.connect('ws://192.168.100.10:8765') as websocket:
    print(f'Client websocket connection is connected to {websocket.host} in port {websocket.port}')
    
    left_cam_idx = 0
    right_cam_idx = 1

    cam_left = cv2.VideoCapture(left_cam_idx)
    cam_right = cv2.VideoCapture(right_cam_idx)

    if not cam_left.isOpened():
      raise Exception('Cannot read Left Camera')
    if not cam_right.isOpened():
      raise Exception('Cannot read Right Camera')
    
    try:
      while(True):
        left_is_captured, left_img = cam_left.read()
        right_is_captured, right_img = cam_right.read()

        h_dsize = round(left_img.shape[0] / 2)
        w_dsize = round(left_img.shape[1]/ 2)

        left_img = cv2.resize(left_img, dsize=(w_dsize, h_dsize))

        # merged_img = np.hstack((left_img, right_img))
        # buffer_img = cv2.imencode('.jpg', merged_img)[1]
        buffer_img = cv2.imencode('.jpg', left_img)[1]

        # first = time.perf_counter()
        await websocket.send(buffer_img.tobytes())
        # last = time.perf_counter()
        # print(first - last)
        time.sleep(.01)
    
    finally:
      cam_left.release()
      cam_right.release()


if __name__ == '__main__':
  print('Client Starting')
  loop = asyncio.get_event_loop()
  loop.run_until_complete(main())