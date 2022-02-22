import platform
from infi.devicemanager import DeviceManager

import cv2
from cv2 import VideoCapture
import numpy as np

def get_camera_index_list(max_cam = 3):
  is_finding = True
  current_cam_index = 0
  cam_list = []

  while (is_finding or current_cam_index < max_cam):
    cam = VideoCapture(current_cam_index, cv2.CAP_DSHOW)
    if cam.isOpened():
      _, img = cam.read()
      np_img = np.array(img)
      print(f'camera index {current_cam_index} camera size {np_img.shape[0]} x {np_img.shape[1]}')
      cam_list.append(current_cam_index)
      current_cam_index += 1
    else:
      is_finding = False
    
  return cam_list

def get_left_right_camera_port(cam_name='Logitech USB Camera (HD Webcam C270)'):
  '''
  Return tuple that value is the index of camera port in order of Left and Right
  '''
  if platform.system() == 'Windows':
    dm = DeviceManager()
    dm.root.rescan()

    device_list =  dm.all_devices
    print(f'number of system devices: {len(device_list)}')

    cam_list = []

    # for device in device_list:
    #   print(device.description)
      # print(device.location)
      # if cam_name in device.description:
      #   print(device.bus_number)
      #   print()


if __name__ == '__main__':
  # get_camera_index_list()
  get_left_right_camera_port()