import cv2
import time
import calibration

class StereoCams():
  def __init__(self, calibrate=False):
    self.left_cam_index = 0
    self.right_cam_index = 1
    self.img = None
    self.cam_left = cv2.VideoCapture(self.left_cam_index)
    self.cam_right = cv2.VideoCapture(self.right_cam_index)
    self.is_calibrate = calibrate

  def read(self, time_split=.01):
    lx, ly, rx, ry = calibration.load_calibrate_map_preset()

    if self.cam_left.isOpened() and self.cam_right.isOpened():
      while True:
        _, left_img = self.cam_left.read()
        _, right_img = self.cam_right.read()

        if self.is_calibrate:
          # 40.1 ms +- 144 microsecond execution time
          left_img = cv2.remap(left_img, lx, ly, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
          right_img = cv2.remap(right_img, rx, ry, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        yield left_img, right_img
        time.sleep(time_split)
  
  def clean_up(self):
    self.cam_left.release()
    self.cam_right.release()
