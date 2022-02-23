print('AAA')

import numpy as np
print('AAA')
import cv2

left_cam_idx = 0
right_cam_idx = 1

print('Reading Camera\n')

cam_left = cv2.VideoCapture(left_cam_idx)
cam_right = cv2.VideoCapture(right_cam_idx)

if not cam_left.isOpened():
  raise Exception('Cannot read Left Camera')
else: 
  print('Left camera is opened')

if not cam_right.isOpened():
  raise Exception('Cannot read Right Camera')
else: 
  print('Right camera is opened')

print('')

left_is_captured, left_img = cam_left.read()
right_is_captured, right_img = cam_right.read()

np_left = np.array(left_img)
np_right = np.array(right_img)
print(f'Left image with size {np_left.shape[0]} x {np_left.shape[1]}')
print(f'Right image with size {np_right.shape[0]} x {np_right.shape[1]}')

cam_left.release()
cam_right.release()
