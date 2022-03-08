from unittest import expectedFailure
import cv2
import numpy as np
from threading import Thread
import asyncio
from websockets.server import serve, WebSocketServerProtocol
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
import os

save_img_dir = os.path.join(os.getcwd(), 'calibration_img')

def setup_img_save_directory():
  if 'calibration_img' not in os.listdir(os.getcwd()):
    os.mkdir(os.path.join(os.getcwd(), 'calibration_img'))
    os.mkdir(os.path.join(os.getcwd(), 'calibration_img', 'left'))
    os.mkdir(os.path.join(os.getcwd(), 'calibration_img', 'right'))

def decode_img_to_byte(img):
  buffer_img = cv2.imencode('.jpg', img)[1]
  return buffer_img.tobytes()

def encode_byte_to_img(img_byte):
  np_img = np.frombuffer(img_byte, np.uint8)
  return cv2.imdecode(np_img, -1)

def safe_frames(left_img, right_img, counter):
  left_img_path = os.path.join(save_img_dir, 'left', 'img_{}.jpg'.format(counter))
  right_img_path = os.path.join(save_img_dir, 'right', 'img_{}.jpg'.format(counter))
  cv2.imwrite(os.path.join(left_img_path), left_img)
  cv2.imwrite(os.path.join(right_img_path), right_img)