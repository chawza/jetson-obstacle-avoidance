from dotenv import load_dotenv
load_dotenv()

import os
import multiprocessing
import time

import SharedArray as sa
import cv2

import calibration
import numpy as np
from depthestimation import DepthEstimator
from robot import Robot

crop_radius = 20
max_n_box = round(480 / crop_radius * 2)
captured_img_size = (480, 640, 3)
object_detect_distance = 2000
closest_object_distance = 500

class RgbColor:
  WHITE = (255, 255, 255)
  BLUE = (255, 0, 0)
  GREEN = (0, 255, 0)
  RED = (0, 0, 255)


class AutonomousRobot(multiprocessing.Process):
  def __init__(self, cam_preset, depth_estimator: DepthEstimator):
    super(AutonomousRobot, self).__init__()
    self.robot = Robot()
    self.cam_preset = cam_preset
    self.depth_estimator = depth_estimator
    self.stop_event = multiprocessing.Event()

    self.left_img = sa.attach('left')
    self.right_img = sa.attach('right')
    self.sa_disparity_img = sa.attach('disparity_img')
    self.sa_distance_list = sa.attach('distance_list')
    self.sa_robot_event = sa.attach('robot_event')

  def run(self):
    print('Autonomous Robot: Starts')
    while(not self.stop_event.is_set()):
      self.robot.stop()
      time.sleep(2)

      gray_left, gray_right = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY), cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY)
      cal_left, cal_right = calibration.calibrate_imgs(gray_left, gray_right, preset=self.cam_preset)

      disparity = self.depth_estimator.get_disparity(cal_left, cal_right)
      self.sa_disparity_img[:] = disparity

      min_distance_list = self.detect_obstacle(disparity)
      # self.sa_distance_list[:] = min_distance_list

      if self.sa_robot_event[2] == 1:
        self.obstacle_avoidance(min_distance_list, debug=True)
      
    self.robot.quit()

  def stop(self):
    self.stop_event.set()
    print('Autonomous Robot: Stops')

  def detect_obstacle(self, disparity):
    img_height = disparity.shape[0]
    img_width = disparity.shape[1]

    y_offset = 50 # move scanline down relative to center
    middle_y, _ = round(img_height / 2), round(img_width / 2)
    middle_y += y_offset

    min_scan_value = []
    max_disparity_value = []
    for x_top_left in range(0, img_width, crop_radius * 2):
      cropped_disparity = disparity[middle_y - crop_radius: middle_y + crop_radius, x_top_left: x_top_left + crop_radius * 2]
      depth_map = self.depth_estimator.predict_depth(cropped_disparity)
      min_distance = np.min(depth_map)
      min_scan_value.append(min_distance)

      max_disparity_in_crop = np.max(cropped_disparity)
      max_disparity_value.append(max_disparity_in_crop)

    return min_scan_value, max_disparity_value

  def draw_obstacle_box(disparity_cmap, min_distance_list):
    img_height = disparity_cmap.shape[0]
    img_width = disparity_cmap.shape[1]

    y_offset = 50
    middle_y, _ = round(img_height / 2), round(img_width / 2)
    middle_y += y_offset

    middle_idx = len(min_distance_list) // 2 
    
    for index, min_scan_value in enumerate(min_distance_list):
      top_left_x = index * crop_radius * 2
      bottom_right_x = (index * crop_radius * 2) + (crop_radius * 2)
      top_left = (
        (top_left_x),
        (middle_y - crop_radius)
      )
      bottom_right = (
        (bottom_right_x),
        (middle_y + crop_radius)
      )

      # always draw middle square
      if index == middle_idx:
        disparity_cmap = cv2.rectangle(disparity_cmap, top_left, bottom_right, RgbColor.GREEN, 3)

      if min_scan_value < closest_object_distance:
        disparity_cmap = cv2.rectangle(disparity_cmap, top_left, bottom_right, RgbColor.RED, 3)
      elif min_scan_value < object_detect_distance:
        disparity_cmap = cv2.rectangle(disparity_cmap, top_left, bottom_right, RgbColor.WHITE, 3)


    return disparity_cmap

  def obstacle_avoidance(self, min_distance_list, debug=False):
    bot_speed = 40
    min_distance_list = np.array(min_distance_list)
    n_box = len(min_distance_list)
    scan_length = n_box / 2

    middle_depth = min_distance_list[round(len(min_distance_list) / 2)]

    if min_distance_list.min() <= closest_object_distance:
      if debug:
        print(f'ROBOT: STOP\t{min_distance_list.min()}')
      self.robot.stop()
      time.sleep(1)
      return

    left_index = None
    right_index = None

    for index, object_min_dist in enumerate(min_distance_list, 1):
      if object_min_dist < object_detect_distance:
        if left_index is None:
          left_index = index
          right_index = index
        else:
          right_index = index

    if left_index is None:
      if debug:
        print(f'FORWARD\t{middle_depth}')
      self.robot.forward(set_speed=bot_speed)
      time.sleep(.5)
      return

    left_split = left_index
    right_split = n_box - right_index

    if left_split <= right_split:
      if right_index > scan_length:
        turn_dir = right_index - scan_length
        turn_dir = (turn_dir / scan_length) * 100
      else:
        turn_dir = 20

      turn_dir += (turn_dir * 50/100)
      
      if debug:
        print(f'RIGHT\t{right_index}\t{turn_dir}%')
      self.robot.forward(set_speed=bot_speed)
      self.robot.right(set_turn_dir=abs(turn_dir))
      time.sleep(.75)
    elif left_split > right_split:
      if left_index <= scan_length:
        turn_dir = scan_length - left_index + 1
        turn_dir = (turn_dir / scan_length) * 100
      else:
        turn_dir = 20

      if debug:
        print(f'LEFT\t{left_index}\t{turn_dir}%')
      self.robot.forward(set_speed=bot_speed)
      self.robot.left(set_turn_dir=abs(turn_dir))
      time.sleep(.75)

