import cv2
import numpy as np
import os
import presetloader

default_save_img_dir = os.path.join(os.getcwd(), 'calibration_img')

def setup_img_save_directory():
  if 'calibration_img' not in os.listdir(os.getcwd()):
    os.mkdir(default_save_img_dir)
    os.mkdir(os.path.join(default_save_img_dir, 'left'))
    os.mkdir(os.path.join(default_save_img_dir, 'right'))

def decode_img_to_byte(img):
  buffer_img = cv2.imencode('.jpg', img)[1]
  return buffer_img.tobytes()

def encode_byte_to_img(img_byte):
  np_img = np.frombuffer(img_byte, np.uint8)
  return cv2.imdecode(np_img, -1)

def encode_img_binary_to_byte(binary_img):
  img = binary_img * 255
  img = img.astype(np.dtype('uint8'))
  return decode_img_to_byte(img)
  
def safe_frames(left_img, right_img, counter, img_dir=default_save_img_dir):
  retL, _ = cv2.findChessboardCorners(left_img, (9,6), None)
  retR, _ = cv2.findChessboardCorners(right_img, (9,6), None)

  if retL is False or retR is False:
    raise RuntimeError('Unable to find corners in one of image image')

  left_img_path = os.path.join(img_dir, 'left', 'img_{}.jpg'.format(counter))
  right_img_path = os.path.join(img_dir, 'right', 'img_{}.jpg'.format(counter))
  cv2.imwrite(left_img_path, left_img)
  cv2.imwrite(right_img_path, right_img)

def initiate_img_counter():
  dir_path = os.path.join(default_save_img_dir, 'left')
  return len(os.listdir(dir_path))

def get_img_path_list():
  left_dir_path = os.path.join(default_save_img_dir, 'left')
  right_dir_path = os.path.join(default_save_img_dir, 'right')

  left_img_filepath = [os.path.join(left_dir_path, fname) for fname in os.listdir(left_dir_path)]
  right_img_filepath = [os.path.join(right_dir_path, fname) for fname in os.listdir(right_dir_path)]

  return left_img_filepath, right_img_filepath

def check_board_calibration():
  # source: https://github.com/niconielsen32/ComputerVision/blob/master/stereoVisionCalibration/stereovision_calibration.py
  chessboard_square_length_mm = 26
  chessboard_corner_size = (9, 6)
  frame_shape = (640, 480)
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

  objp = np.zeros((chessboard_corner_size[0] * chessboard_corner_size[1], 3), np.float32)
  objp[:,:2] = np.mgrid[0:chessboard_corner_size[0],0:chessboard_corner_size[1]].T.reshape(-1,2)
  objp = objp * chessboard_square_length_mm

  objpoints = []
  corner_points_left = []
  corner_points_right = []

  left_path_list, right_path_list = get_img_path_list()

  for left_path, right_path in zip(left_path_list, right_path_list):
    left_img = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    right_img = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
    
    retL, cornersLeft = cv2.findChessboardCorners(left_img, chessboard_corner_size, None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    retR, cornersRight = cv2.findChessboardCorners(right_img, chessboard_corner_size, None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if retL and retR:
      objpoints.append(objp)

      corner_sp_l = cv2.cornerSubPix(left_img, cornersLeft, (11,11), (-1, -1), criteria)
      corner_sp_r = cv2.cornerSubPix(right_img, cornersRight, (11,11), (-1, -1), criteria)

      corner_points_left.append(corner_sp_l)
      corner_points_right.append(corner_sp_r)

    else:
      print('Cannot find corners in {}'.format(left_path))
  
  if len(objpoints) < 1:
    raise RuntimeError('Unable to find corners in all images')
  
  flags = 0
  flags |= cv2.CALIB_FIX_INTRINSIC

  # Camera Callibration
  retL, camera_matrix_left, dist_l, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, corner_points_left, frame_shape, None, None)
  new_camera_matrix_l, _  = cv2.getOptimalNewCameraMatrix(camera_matrix_left, dist_l, frame_shape, 1, frame_shape)
  
  retR, camera_matrix_right, dist_r, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, corner_points_right, frame_shape, None, None)
  new_camera_matrix_r, _ = cv2.getOptimalNewCameraMatrix(camera_matrix_right, dist_r, frame_shape, 1, frame_shape)
  
  # Stereo Calibration
  retStereo, new_camera_matrix_l, dist_l, new_camera_matrix_r, dist_r, rot, trans, essenstial_matrix, fundamental_matrix = cv2.stereoCalibrate(
    objpoints, corner_points_left, corner_points_right, new_camera_matrix_l, dist_l, new_camera_matrix_r, dist_r, frame_shape, flags=flags, criteria=criteria
  )

  rectL, rectR, project_matrix_l, project_matrix_r, Q, roi_L, roi_R = cv2.stereoRectify(
    new_camera_matrix_l, dist_l, new_camera_matrix_r, dist_r, frame_shape, rot, trans
  )

  stereoMapL = cv2.initUndistortRectifyMap(new_camera_matrix_l, dist_l, rectL, project_matrix_l, frame_shape, m1type=cv2.CV_16SC2)
  stereoMapR = cv2.initUndistortRectifyMap(new_camera_matrix_r, dist_r, rectR, project_matrix_r, frame_shape, m1type=cv2.CV_16SC2)

  return stereoMapL, stereoMapR

def calibrate_imgs(left, right, preset):
  (left_x, left_y), (right_x, right_y) = preset
  calibrated_left = cv2.remap(left, left_x, left_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, 0)
  calibrated_right = cv2.remap(right, right_x, right_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, 0)
  return calibrated_left, calibrated_right
  
def calibrate_cam():
  print('Camera Calibrating')
  stereoMapL, stereoMapR = check_board_calibration()
  presetloader.save_calibration_map_preset(stereoMapL, stereoMapR)
  print('Done camera calibration')

class CalibrateSession():
  def __init__(self, initial_img_indx = None, save_img_dir=default_save_img_dir):
    self.img_index = initial_img_indx or initiate_img_counter()
    self.img_dir = save_img_dir
    
    self.depth_mapping_list = []
    self.defult_depth_map_file_name = 'depth_map'

  def capture_frame(self, left, right):
    try:
      safe_frames(left, right, self.img_index, img_dir=self.img_dir)
      print('Saving Frame {}'.format(self.img_index))
      self.img_index += 1
    except RuntimeError as error:
      print(error)
  
  def calibrate(self):
    calibrate_cam()

  def add_depth_map(self, disparity, depth):
    self.depth_mapping_list.append([disparity, depth])

  def save_depth_map(self):
    presetloader.save_depth_map(self.depth_mapping_list)

if __name__ == '__main__':
  import sys
  
  if '--calibrate' in sys.argv or '-c' in sys.argv:
    calibrate_cam()
