import cv2
import numpy as np

import calibration

class DepthEstimator():
  def __init__(self, cam_preset, cam_focal = .4, cam_baseline = 71, numDisparities=96, blockSize=39, minDisparity=0):
    stereo = cv2.StereoBM_create()
    stereo.setNumDisparities(numDisparities)
    stereo.setMinDisparity(minDisparity)
    stereo.setBlockSize(blockSize)

    self.stereo = stereo
    self.focal = cam_focal
    self.baseline = cam_baseline
    self.cam_preset =  cam_preset


  def preprocess_img(self, left, right):
    left_img, right_img = calibration.calibrate_imgs(left, right, self.cam_preset)

    if len(left_img.shape) == 3:
      left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
      right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    return left_img, right_img


  # https://learnopencv.com/depth-perception-using-stereo-camera-python-c/
  def get_disparity(self, left_img, right_img):
    left_img, right_img = self.preprocess_img(left_img, right_img)
    disparity = self._block_maching(left_img, right_img)
    return disparity


  def get_depth(self, left_img, right_img, max_depth = 300, min_depth=0):
    disparity = self.get_disparity(left_img, right_img)
    depth = self._estimate_depth(disparity, max_depth, min_depth)
    return depth


  def _block_maching(self, left, right):
    disparity = self.stereo.compute(left, right)
    disparity = disparity.astype(np.float32)
    return disparity / 16

    # https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
  def _estimate_depth(self, disparity, max_depth, min_depth):
    depth =  (self.focal * self.baseline) / disparity
    # filter out of range object
    depth[depth < min_depth] = min_depth
    depth[depth > max_depth] = max_depth
    return depth


  def normalize_disparity(self, disparity):
    norm_disp = (disparity - self.stereo.getMinDisparity()) / self.stereo.getNumDisparities()
    return norm_disp
