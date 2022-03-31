import cv2
import numpy as np
import calibration
from capture_cam import StereoCams
import os

calibration.setup_img_save_directory()
counter = calibration.initiate_img_counter()
cam_preset = calibration.load_calibrate_map_preset()
is_calibrate = False
camera = StereoCams(left_idx=0, right_idx=2)

for left, right in camera.read():
  right = cv2.flip(right, -1)
  if is_calibrate:
    left, right = calibration.calibrate_imgs(left, right, cam_preset)

  img = np.hstack((left, right))

  cv2.imshow('img', img)

  key = cv2.waitKey(10)
  if key == ord('q'):
    break
  if key == ord(' '):
    calibration.safe_frames(left, right, counter)
    print('saved : {}'.format(counter))
    counter += 1
  if key == ord('c'):
    is_calibrate = not is_calibrate
    print(f'Calibrate: {is_calibrate}')
