import cv2
import time
import calibration

class StereoCams():
  def __init__(self):
    self.left_cam_index = 0
    self.right_cam_index = 1
    self.img = None
    self.cam_left = cv2.VideoCapture(self.left_cam_index)
    self.cam_right = cv2.VideoCapture(self.right_cam_index)

  def read(self, time_split=.01):
    if self.cam_left.isOpened() and self.cam_right.isOpened():
      while True:
        _, left_img = self.cam_left.read()
        _, right_img = self.cam_right.read()

        yield left_img, right_img
        time.sleep(time_split)
  
  def clean_up(self):
    self.cam_left.release()
    self.cam_right.release()
