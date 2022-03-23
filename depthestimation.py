import os
import json
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
    self.stereo_preset_filename = 'stereo_preset'


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

  def save_current_preset(self, param_dir = None):
    if param_dir == None:
      param_dir = os.path.join(os.path.dirname(__file__), 'stereo presets')

    sbm = self.stereo
    params = {
      'NumDisparities':  sbm.getNumDisparities(),
      'BlockSize': sbm.getBlockSize(), 
      'MinDisparity': sbm.getMinDisparity(),
      'PreFilterType': sbm.getPreFilterType(),
      'PreFilterSize': sbm.getPreFilterSize(),
      'PreFilterCap': sbm.getPreFilterCap(),
      'TextureThreshold': sbm.getTextureThreshold(),
      'UniquenessRatio': sbm.getUniquenessRatio(),
      'SpeckleRange': sbm.getSpeckleRange(),
      'SpeckleWindowSize': sbm.getSpeckleWindowSize(),
      'Disp12MaxDiff': sbm.getDisp12MaxDiff()
    }

    preset_idx = self.reserve_stereo_preset_index() + 1

    file_path = os.path.join(param_dir, f'{self.stereo_preset_filename}_{preset_idx}.json')
    with open(file_path, 'w') as file:
      json.dump(params, file)


  def load_current_preset(self, file_path=None):
    sbm = self.stereo

    if file_path is None:
      project_dir = os.path.dirname(__file__)
      preset_dir = os.path.join(project_dir, 'stereo presets')
      preset_list = os.listdir(preset_dir)
      preset_list = [file_name for file_name in preset_list if file_name.endswith('.json')]
      if len(preset_list) == 0:
        raise FileNotFoundError('preset not found in {}'.format(preset_dir))
      preset_list = sorted(preset_list)
      file_path = preset_list[-1]      

    with open(file_path, 'r') as file:
      loaded_preset = json.load(file)
      if len(loaded_preset) < 1:
        raise RuntimeError('Cannot load stereo preset')
    
    for key, value in loaded_preset.items():
      try:
        set_func = getattr(sbm, 'set{}'.format(key))
        set_func(int(value))
      except Exception as err:
        print('Unable to set {} to {}'.format(value, key))
        raise err

  def reserve_stereo_preset_index(current = False):
    stereo_preset_dir = os.path.join(os.path.dirname(__file__), 'stereo presets')
    files = os.listdir(stereo_preset_dir)
    if current:
      return len(files)
    return len(files) + 1


      
