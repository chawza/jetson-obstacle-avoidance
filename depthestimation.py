import os

import cv2
import numpy as np

import calibration

class DepthEstimator():
  def __init__(self, cam_focal = .4, cam_baseline = 7.3, numDisparities=96, blockSize=39, minDisparity=0):
    stereo = cv2.StereoBM_create()
    stereo.setNumDisparities(numDisparities)
    stereo.setMinDisparity(minDisparity)
    stereo.setBlockSize(blockSize)

    self.stereo = stereo
    self.focal = cam_focal
    self.baseline = cam_baseline


  def estimate(self, left_img, right_img):
    if len(left_img.shape) == 3:
      left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
      right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

    left_img, right_img = calibration.calibrate_imgs(left_img, right_img)
    disparity = self._block_maching(left_img, right_img) # 128 ms ± 277 µs per loop (mean ± std. dev. of 7 runs, 10 loops each)
    depth = self._estimate_depth(disparity) # 135 ms ± 481 µs per loop (mean ± std. dev. of 7 runs, 10 loops each)
    return depth


  def _block_maching(self, left, right):
    disparity = self.stereo.compute(left, right)
    disparity = (disparity/16.0 - self.stereo.getMinDisparity()) / self.stereo.getNumDisparities()
    return disparity


  def _estimate_depth(self, disparity):
    M = (self.focal * self.baseline)
    depth =  np.divide(M, disparity, where=disparity != 0)
    return depth


    
