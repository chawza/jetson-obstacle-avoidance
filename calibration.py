import cv2
import numpy as np
import os

save_img_dir = os.path.join(os.getcwd(), 'calibration_img')

def setup_img_save_directory():
  if 'calibration_img' not in os.listdir(os.getcwd()):
    os.mkdir(save_img_dir)
    os.mkdir(os.path.join(save_img_dir, 'left'))
    os.mkdir(os.path.join(save_img_dir, 'right'))

def decode_img_to_byte(img):
  buffer_img = cv2.imencode('.jpg', img)[1]
  return buffer_img.tobytes()

def encode_byte_to_img(img_byte):
  np_img = np.frombuffer(img_byte, np.uint8)
  return cv2.imdecode(np_img, -1)

def safe_frames(left_img, right_img, counter):
  left_img_path = os.path.join(save_img_dir, 'left', 'img_{}.jpg'.format(counter))
  right_img_path = os.path.join(save_img_dir, 'right', 'img_{}.jpg'.format(counter))
  cv2.imwrite(left_img_path, left_img)
  cv2.imwrite(right_img_path, right_img)

def initiate_img_counter():
  dir_path = os.path.join(save_img_dir, 'left')
  return len(os.listdir(dir_path))

def get_img_path_list():
  left_dir_path = os.path.join(save_img_dir, 'left')
  right_dir_path = os.path.join(save_img_dir, 'right')

  left_img_filepath = [os.path.join(left_dir_path, fname) for fname in os.listdir(left_dir_path)]
  right_img_filepath = [os.path.join(right_dir_path, fname) for fname in os.listdir(right_dir_path)]

  return left_img_filepath, right_img_filepath

def calculate_stereo_map():
  # source: https://github.com/niconielsen32/ComputerVision/blob/master/stereoVisionCalibration/stereovision_calibration.py
  chessboard_square_length_mm = 17
  chessboard_corner_size = (10, 6)
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
    
    retL, cornersLeft = cv2.findChessboardCorners(left_img, chessboard_corner_size, None)
    retR, cornersRight = cv2.findChessboardCorners(right_img, chessboard_corner_size, None)

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

  retL, camera_matrix_left, dist_l, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, corner_points_left, frame_shape, None, None)
  new_camera_matrix_l, roi_l = cv2.getOptimalNewCameraMatrix(camera_matrix_left, dist_l, frame_shape, 1)
  retR, camera_matrix_right, dist_r, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, corner_points_right, frame_shape, None, None)
  new_camera_matrix_r, roi_r = cv2.getOptimalNewCameraMatrix(camera_matrix_right, dist_l, frame_shape, 1)
  
  retStereo, new_camera_matrix_l, dist_L, new_camera_matrix_r, dist_R, rot, trans, essenstial_matrix, fundamental_matrix = cv2.stereoCalibrate(
    objpoints, corner_points_left, corner_points_right, new_camera_matrix_l, dist_l, new_camera_matrix_r, dist_r, frame_shape, criteria, flags
  )

  rectL, rectR, project_matrix_l, projecet_matrix_r, Q, roi_L, roi_R = cv2.stereoRectify(
    new_camera_matrix_l, dist_l, new_camera_matrix_r, dist_r, frame_shape, rot, trans, 1, (0,0)
  )

  stereoMapL = cv2.initUndistortRectifyMap(new_camera_matrix_l, dist_l, rectL, project_matrix_l, frame_shape, cv2.CV_16SC2)
  stereoMapR = cv2.initUndistortRectifyMap(new_camera_matrix_r, dist_l, rectR, projecet_matrix_r, frame_shape, cv2.CV_16SC2)
    
  return stereoMapL, stereoMapR

def save_calibration_map_preset(stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y):
  preset_dir = os.path.join(os.getcwd(), 'calibration_preset')
  
  np.save(os.path.join(preset_dir, 'stereoMapL_x.npy'), stereoMapL_x)
  np.save(os.path.join(preset_dir, 'stereoMapL_y.npy'), stereoMapL_y)
  np.save(os.path.join(preset_dir, 'stereoMapR_x.npy'), stereoMapR_x)
  np.save(os.path.join(preset_dir, 'stereoMapR_y.npy'), stereoMapR_y)

def load_calibrate_map_preset(preset_path = None):
  if preset_path:
    preset_dir = preset_path
  else:
    preset_dir = os.path.join(os.getcwd(), 'calibration_preset')

  stereoMapL_x = np.load(os.path.join(preset_dir, 'stereoMapL_x.npy'))
  stereoMapL_y = np.load(os.path.join(preset_dir, 'stereoMapL_y.npy'))
  stereoMapR_x = np.load(os.path.join(preset_dir, 'stereoMapR_x.npy'))
  stereoMapR_y = np.load(os.path.join(preset_dir, 'stereoMapR_y.npy'))

  return stereoMapL_x, stereoMapL_y, stereoMapR_x, stereoMapR_y

def calibrate_imgs(left, right, preset_path):
  left_x, left_y, right_x, right_y = load_calibrate_map_preset(preset_path)
  
  calibrated_left = cv2.remap(left, left_x, left_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
  calibnrated_right = cv2.remap(right, right_x, right_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

  return calibrated_left, calibnrated_right
  
def calibrate_cam():
  print('Camera Calibrating')
  stereoMapL, stereoMapR = calculate_stereo_map()
  save_calibration_map_preset(stereoMapL[0], stereoMapL[1], stereoMapR[0], stereoMapR[1])
  print('Done camera calibration')
