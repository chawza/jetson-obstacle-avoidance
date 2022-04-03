import os
import json
import cv2
import numpy as np
import joblib

from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline
import presetloader
from dataclasses import dataclass

@dataclass
class DepthParameter():
    near_depth: float
    medium_depth: float
    far_trehsold: float

default_depth_param = DepthParameter(300, 600, 900)
class DepthEstimator():
  def __init__(self, depth_param = default_depth_param, cam_focal = .4, cam_baseline = 38, max_depth=3000, min_depth=0, frame_size = (640, 480)):
    # class variables
    self.stereo = cv2.StereoBM_create()
    self.stereo_preset_filename = 'stereo_preset'
    self.depth_param = depth_param

    # depth calculation parameter
    self.max_depth = max_depth
    self.min_depth = min_depth
    
    self.focal_length_in_px = (frame_size[0] * .5) / np.tan(60 * .5 * np.pi/180) # https://stackoverflow.com/a/38109434/10471394
    # other calucation: https://support.geocue.com/converting-focal-length-from-pixels-to-millimeters-to-use-in-bentley-context-capture/#:~:text=F(mm)%20%3D%20F(,width%20in%20pixels%20is%205472.

    self.baseline = cam_baseline
    self.M = self.baseline * self.focal_length_in_px

    # Depth estimation model
    self.depth_prediction_model = Pipeline([
      ('poly', PolynomialFeatures(degree=3)),
      ('lienar', LinearRegression())
    ])

  def _block_maching(self, left, right):
    disparity = self.stereo.compute(left, right)
    disparity = disparity.astype(np.float32)
    return disparity / 16

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

    # https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
  def _estimate_depth(self, disparity):
    depth =  self.M / disparity

    # filter out of range object
    depth[depth < self.min_depth] = self.min_depth
    depth[depth > self.max_depth] = self.max_depth
    return depth

  # utils
  def normalize_disparity(self, disparity):
    norm_disp = (disparity - self.stereo.getMinDisparity()) / self.stereo.getNumDisparities()
    return norm_disp
  
  def disparity_to_gray(self, disparity):
    norm_disparity = self.normalize_disparity(disparity)
    return (norm_disparity * 255).astype(np.dtype('uint8'))
  
  def disparity_to_colormap(self, disparity):
    norm_disparity = self.normalize_disparity(disparity)
    int_disp = (norm_disparity * 255).astype(np.dtype('uint8'))
    return cv2.applyColorMap(int_disp, cv2.COLORMAP_JET)

  def normalize_depth(self, depth, reverse = False):
    norm_depth = (depth - self.min_depth) / (self.max_depth - self.min_depth)
    if reverse:
      norm_depth = 1 - norm_depth
    return norm_depth

  def depth_to_colormap(self, depth):
    norm_depth = self.normalize_depth(depth, reverse=True)
    int_depth = (norm_depth * 255).astype(np.dtype('uint8'))
    return cv2.applyColorMap(int_depth, cv2.COLORMAP_JET)

  # SBM model
  def get_all_sbm_properties(self):
    sbm = self.stereo
    return {
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


  def save_current_preset(self, param_dir = None):
    if param_dir == None:
      param_dir = os.path.join(os.path.dirname(__file__), 'stereo presets')

    params = self.get_all_sbm_properties()
    presetloader.save_sbm_preset_json(params)

  def load_preset(self, filepath=None):
    sbm = self.stereo
    if filepath:
      loaded_preset = presetloader.load_sbm_preset_json(filepath)
    else:
      loaded_preset = presetloader.load_sbm_preset_json()
      
    for key, value in loaded_preset.items():
      try:
        print('set{} to {}'.format(key, value))
        set_func = getattr(sbm, 'set{}'.format(key))
        set_func(int(value))
      except Exception as err:
        print('Unable to set {} to {}'.format(value, key))
        raise err

  # Depth prediction
  def train_depth_mapping(self, depth_map_filepath=None):
    depth_map = presetloader.load_depth_map(depth_map_filepath)
    # make it sparse
    disparities = [[value[0]] for value in depth_map]
    depth_in_mm = [[value[1]] for value in depth_map]

    disparities = np.float32(disparities)
    depth_in_mm = np.float32(depth_in_mm)

    self.depth_prediction_model.fit(disparities, depth_in_mm)    
    
  def predict_depth(self, disparity):
    if isinstance(disparity, np.ndarray):
      input_shape = disparity.shape
      disparity = disparity.flatten().reshape(-1, 1)
      depth =  self.depth_prediction_model.predict(disparity)
      return depth.reshape(input_shape)
    
    # singe value
    result = self.depth_prediction_model.predict([[disparity]])
    return result[0][0]
