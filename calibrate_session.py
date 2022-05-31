import os
from dotenv import load_dotenv
load_dotenv()

import cv2
import numpy as np
import calibration
from capture_cam import StereoCams
from depthestimation import DepthEstimator
import presetloader

SBM_PRESET_FILE = os.getenv('SBM_PRESET_FILE')

is_calibrate = True
calibration.setup_img_save_directory()
try:
  cam_preset = presetloader.load_calibrate_map_preset()
except FileNotFoundError:
  is_calibrate = False
camera = StereoCams(left_idx=0, right_idx=2)


def stereo_calibration_session():
  global is_calibrate
  estimator = DepthEstimator()
  estimator.load_preset(SBM_PRESET_FILE)
  counter = calibration.initiate_img_counter()
  test_disparity = False
  for left, right in camera.read():
    if is_calibrate:
      cal_left, cal_right = calibration.calibrate_imgs(left, right, cam_preset)
      img = np.hstack((cal_left, cal_right))
    elif test_disparity:
      cal_left, cal_right = calibration.calibrate_imgs(left, right, cam_preset)
      left_gray = cv2.cvtColor(cal_left, cv2.COLOR_BGR2GRAY)
      right_gray = cv2.cvtColor(cal_right, cv2.COLOR_BGR2GRAY)
      disparity = estimator.get_disparity(left_gray, right_gray)
      img = estimator.disparity_to_colormap(disparity)
    else:
      img = np.hstack((left, right))

    cv2.imshow('img', img)

    key = cv2.waitKey(10)
    if key == ord('q'):
      break
    if key == ord(' '):
      try:
        calibration.safe_frames(left, right, counter)
        print('saved : {}'.format(counter))
        counter += 1
      except RuntimeError as err:
        print(err)
    if key == ord('c'):
      is_calibrate = True
      test_disparity = not is_calibrate

      print(f'Calibrate: {is_calibrate}')

    if key == ord('d'):
      test_disparity = True
      is_calibrate = not test_disparity


  camera.clean_up()

def depth_calibration_session():
  global camera
  window_name = 'disparity'
  cv2.namedWindow(window_name)
  estimator = DepthEstimator()
  estimator.load_preset(SBM_PRESET_FILE)
  estimator.train_depth_mapping()
  session = calibration.CalibrateSession()
  
  def mouse_call_back(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
      disp = param['disparity']
      disp_value = disp[y][x]
      real_world_depth = input(f'Disparity {disp_value} Depth value (mm): ')
      if not real_world_depth.isnumeric():
        print('input {} is not numeric'.format(real_world_depth))
        return
      session.add_depth_map(np.float32(disp_value), np.float32(real_world_depth ))

  for left, right in camera.read(time_split=.01):
    left, right = calibration.calibrate_imgs(left, right, cam_preset)
    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    disp = estimator.get_disparity(left_gray, right_gray)
    disp_jet_map = estimator.disparity_to_colormap(disp)

    cv2.imshow(window_name, disp_jet_map)
    cv2.setMouseCallback(window_name, mouse_call_back, param={'disparity': disp})

    key = cv2.waitKey(10)
    if ord('q') == key:
      break
    if ord('v') == key:
      print(session.depth_mapping_list)
      print('length: ', len(session.depth_mapping_list))
    if ord('s') == key:
      print('saving')
      session.save_depth_map()
  
  print('out fo cam read')
  camera.clean_up()

if __name__ == '__main__':
  import sys

  if '-d' in sys.argv or '--depth-map' in sys.argv:
    depth_calibration_session()
  else:
    stereo_calibration_session()