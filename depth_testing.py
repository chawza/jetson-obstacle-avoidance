import cv2
import calibration
from capture_cam import StereoCams
import presetloader
import time
import numpy as np

is_calibrate = True
calibration.setup_img_save_directory()
try:
  cam_preset = presetloader.load_calibrate_map_preset()
except FileNotFoundError:
  print('unable to load camera setting')
  is_calibrate = False
camera = StereoCams(left_idx=0, right_idx=2)

def depth_estimation():
  global camera
  from depthestimation import DepthEstimator
  window_name = 'disparity'
  cv2.namedWindow(window_name)
  estimator = DepthEstimator()
  estimator.load_preset()
  estimator.train_depth_mapping()
    
  while True:
    left, right = camera.read_once()
    left, right = calibration.calibrate_imgs(left, right, cam_preset)
    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    disp = estimator.get_disparity(left_gray, right_gray)
    disp_jet_map = estimator.disparity_to_colormap(disp)

    middle_y, middle_x = round(disp.shape[0] / 2), round(disp.shape[1] / 2)
    crop_radius = 10

    # img_with_circle = cv2.circle(disp_jet_map, (middle_x, middle_y), 3, (255, 255, 255), cv2.FILLED)
    crop_disparity = disp[middle_y - crop_radius: middle_y + crop_radius, middle_x - crop_radius: middle_x + crop_radius]
    depth_map = estimator.predict_depth(crop_disparity)
    # print('max: ', np.max(depth_list))
    print('min', np.min(depth_map))
    # print('avg', np.average(depth_list), end='\n\n')
    # print('std:\t', np.std(depth_map))
    # print('depth (mm): ', np.min(crop_depth))

    img_with_sqare = cv2.rectangle(disp_jet_map, (middle_x - crop_radius, middle_y - crop_radius), (middle_x + crop_radius, middle_y + crop_radius), (255, 255, 255), cv2.FILLED)
    cv2.imshow('disparity', img_with_sqare)

    key = cv2.waitKey(10)
    if ord('q') == key:
      break

    time.sleep(.01)

  print('out fo cam read')
  camera.clean_up()

if __name__ == '__main__':
    depth_estimation()