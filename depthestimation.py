import os
import json
import cv2
import numpy as np

import calibration

class DepthEstimator():
  def __init__(self, cam_focal = .4, cam_baseline = 38, max_depth=3000, min_depth=0, frame_size = (640, 480)):
    # class variables
    self.stereo = cv2.StereoBM_create()
    self.stereo_preset_filename = 'stereo_preset'

    # depth calculation
    self.max_depth = max_depth
    self.min_depth = min_depth
    
    self.focal_length_in_px = (frame_size[0] * .5) / np.tan(60 * .5 * np.pi/180) # https://stackoverflow.com/a/38109434/10471394
    # other calucation: https://support.geocue.com/converting-focal-length-from-pixels-to-millimeters-to-use-in-bentley-context-capture/#:~:text=F(mm)%20%3D%20F(,width%20in%20pixels%20is%205472.

    self.baseline = cam_baseline
    self.M = self.baseline * self.focal_length_in_px

  # https://learnopencv.com/depth-perception-using-stereo-camera-python-c/
  def get_disparity(self, left_img, right_img):
    if len(left_img.shape) > 2:
      raise ValueError('Input images should be one dimension (grayscale)')
    disparity = self._block_maching(left_img, right_img)

    # filter
    minDisp = self.stereo.getMinDisparity()
    numDisp = self.stereo.getNumDisparities()
    disparity[disparity < minDisp] = minDisp
    disparity[disparity > numDisp] = numDisp

    return disparity


  def get_depth(self, left_img, right_img):
    disparity = self.get_disparity(left_img, right_img)
    depth = self._estimate_depth(disparity)
    return depth


  def _block_maching(self, left, right):
    disparity = self.stereo.compute(left, right)
    disparity = disparity.astype(np.float32)
    return disparity / 16

    # https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
  def _estimate_depth(self, disparity):
    depth =  self.M / disparity

    # filter out of range object
    depth[depth < self.min_depth] = self.min_depth
    depth[depth > self.max_depth] = self.max_depth
    return depth


  def normalize_disparity(self, disparity):
    norm_disp = (disparity - self.stereo.getMinDisparity()) / self.stereo.getNumDisparities()
    return norm_disp
  
  def disparity_to_colormap(self, disparity):
    norm_disparity = self.normalize_disparity(disparity)
    int_disp = (norm_disparity * 255).astype(np.dtype('uint8'))
    return cv2.applyColorMap(int_disp, cv2.COLORMAP_JET)

  def normalize_depth(self, depth, reverse = False):
    norm_depth = (depth - self.min_depth) / (self.max_depth - self.min_depth)
    if reverse:
      norm_depth = 1 - norm_depth
    return norm_depth

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

    file_path = os.path.join(param_dir, f'{preset_idx}_{self.stereo_preset_filename}.json')
    with open(file_path, 'w') as file:
      json.dump(params, file)


  def load_preset(self, file_path=None):
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
