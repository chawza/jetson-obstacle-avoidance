from glob import glob
from threading import Thread
from getch import getch
import time

import numpy as np
import cv2

import calibration
import presetloader
from capture_cam import StereoCams
from robot import Robot
from depthestimation import DepthEstimator

bot = Robot()
camera = StereoCams()
cam_preset = presetloader.load_calibrate_map_preset()
estimator = DepthEstimator()
estimator.load_preset('/home/nabeel/jetson-obstacle-avoidance/preset/1_sbm_param.json')
estimator.load_depth_model()

def obstacle_avodance():
  print('auto mode')
  crop_radius = 40
  y_offset = 50
  img_shape = (640, 480)
  middle_y, middle_x = round(img_shape[1] / 2), round(img_shape[0] / 2)
  middle_y += y_offset
  scan_box_width = round(640 / 2 / 6)
  right_x_index = [x for x in range(middle_x, img_shape[0], scan_box_width)]
  left_x_index = [x for x in range(144, middle_x, scan_box_width)]
  left_x_index.reverse()
  left_x_index = np.append(left_x_index, [left_x_index[-1] for _ in range(len(right_x_index) - len(left_x_index))])

  while True:
    left_img, right_img = camera.read_once()
    cal_left, cal_right = calibration.calibrate_imgs(left_img, right_img, preset=cam_preset)
    left_gray = cv2.cvtColor(cal_left, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(cal_right, cv2.COLOR_BGR2GRAY)

    disparity = estimator.get_disparity(left_gray, right_gray)
    

    crop_disparity = disparity[middle_y - crop_radius: middle_y + crop_radius, middle_x - crop_radius: middle_x + crop_radius]
    depth_map = estimator.predict_depth(crop_disparity)

    center_min_depth = np.min(depth_map)
    # print('DEPTH\t', min_depth)

    if center_min_depth < 500:
      print('center ', center_min_depth)
      bot.stop()
    
    elif center_min_depth > 2000:
      if bot.speed <= 25:
        bot.forward()

    # 500 < min_depth < 2000
    else:
      for index, (left_x , right_x) in enumerate(zip(left_x_index, right_x_index), start=1):
        # crop the area
        crop_left = disparity[middle_y - crop_radius: middle_y + crop_radius, left_x: left_x + scan_box_width]
        crop_right = disparity[middle_y - crop_radius: middle_y + crop_radius, right_x: right_x + scan_box_width]
        
        # get depth in cropped area
        left_depth = estimator.predict_depth(crop_left)
        crop_depth = estimator.predict_depth(crop_right)

        # calculate difference
        left_min = np.min(left_depth)
        right_min = np.min(crop_depth)
        diff = left_min - right_min
  
        if abs(diff) < 40:
          continue

        if left_min > right_min:
          bot.left(set_turn_dir=-index)
          break
        elif right_min >= left_min:
          bot.right(set_turn_dir=index)
          break
      if bot.speed < 25:
        bot.forward()
    
    bot.print_debug()

def app():
  global bot
  print('Robot Activated')
  while True:
    key = getch()
    
    if key == 'w':
      bot.forward()
    elif key == 's':
      bot.backward()
    elif key == 'x':
      bot.stop()
    elif key == 'd':
      bot.right()
    elif key == 'a':
      bot.left()

    elif key == 'm':
      try:
        obstacle_avodance()
      except KeyboardInterrupt:
        bot.stop()
        print('manual mode')
    
    elif key == 'q':
      bot.rotate_left()
    elif key == 'e':
      bot.rotate_right()
    elif key == 'z':
      break
    bot.print_debug()

  print('Robot Deactivated')

if __name__ == '__main__':
    app()
    camera.clean_up()
    bot.quit()
