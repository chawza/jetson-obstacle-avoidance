import cv2
import time

class StereoCams():
  def __init__(self, left_idx = 0, right_idx = 1, capture_size=(640, 480)):
    self.left_cam_index = left_idx
    self.right_cam_index = right_idx
    self.cam_left = cv2.VideoCapture(self.left_cam_index)
    self.cam_right = cv2.VideoCapture(self.right_cam_index)

    """ Sync the frame grabbing by adjusting the buffer size based on 
    https://stackoverflow.com/questions/30032063/opencv-videocapture-lag-due-to-the-capture-buffer
    NOTE: not fully sync, but much better than before. other method to try
    https://stackoverflow.com/questions/21671139/how-to-synchronize-two-usb-cameras-to-use-them-as-stereo-camera """
    self.cam_left.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    self.cam_right.set(cv2.CAP_PROP_BUFFERSIZE, 8)
    # Other config: left = 1  right = 2

    # set screen grab size
    self.cam_left.set(cv2.CAP_PROP_FRAME_WIDTH, capture_size[0])
    self.cam_left.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_size[1])
    self.cam_right.set(cv2.CAP_PROP_FRAME_WIDTH, capture_size[0])
    self.cam_right.set(cv2.CAP_PROP_FRAME_HEIGHT, capture_size[1])

    self.state = True

  def read(self, time_split=.01):
    # TODO: sync capture image between two camera
    # https://stackoverflow.com/questions/21671139/how-to-synchronize-two-usb-cameras-to-use-them-as-stereo-camera
    if self.cam_left.isOpened() and self.cam_right.isOpened():
      while self.state is True:
        _, left_img = self.cam_left.read()
        _, right_img = self.cam_right.read()

        # right camera is positioned upside-down in order to reduce the stereo basline length
        # fliping Vertically and Horizontally will fix the upside-down image
        right_img = cv2.flip(right_img, -1)

        yield left_img, right_img
        time.sleep(time_split)
  
    self.cam_left.release()
    self.cam_right.release()
    print('VideoCapture released')

  def read_once(self):
    _, left_img = self.cam_left.read()
    _, right_img = self.cam_right.read()
    right_img = cv2.flip(right_img, -1)
    return left_img, right_img

  def clean_up(self):
    self.state = False
    self.cam_left.release()
    self.cam_right.release()
    print('cams stop')
