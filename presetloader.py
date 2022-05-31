import os
import joblib
import json
import numpy as np

project_dir = os.path.dirname(__file__)
default_preset_dir = 'preset'

# camera calibration
def save_calibration_map_preset(stereoMapL, stereoMapR, filename='stereo_preset.pickle'):
  file_path = os.path.join(project_dir, default_preset_dir, filename)
  print(f'saving camera preset in {file_path}')
  stereo_preset = (stereoMapL, stereoMapR)
  joblib.dump(stereo_preset, file_path)

def load_calibrate_map_preset(filename='stereo_preset.pickle'):
  file_path = os.path.join(project_dir, default_preset_dir, filename)
  print('loading stereo camera preset from {}'.format(file_path))
  stereo_preset = joblib.load(file_path)
  stereoMapL, stereoMapR = stereo_preset
  return (stereoMapL[0], stereoMapL[1]), (stereoMapR[0], stereoMapR[1])

# sbm preset
def save_sbm_preset_json(preset, filename='sbm_param.json'):
  index = current_sbm_preset_count() + 1
  file_path = os.path.join(project_dir, default_preset_dir, f'{index}_{filename}')
  with open(file_path, 'w') as file:
    json.dump(preset, file)

def load_sbm_preset_json(filepath=None):
  if filepath is None:
    filepath = os.path.join(project_dir, default_preset_dir, get_latest_sbm_preset_filename())
  with open(filepath, 'r') as file:
    preset_dict = json.load(file)
  return preset_dict

def current_sbm_preset_count():
  preset_dir = os.path.join(project_dir, default_preset_dir)
  filenames = os.listdir(preset_dir)
  sbm_preset_filenames = [filename for filename in filenames if filename.endswith('.json')]
  return len(sbm_preset_filenames)

def get_latest_sbm_preset_filename():
  preset_dir = os.path.join(project_dir, default_preset_dir)
  filenames = os.listdir(preset_dir)
  sbm_preset_filenames = [filename for filename in filenames if filename.endswith('.json')]
  sbm_preset_filenames = sorted(sbm_preset_filenames)
  if len(sbm_preset_filenames) == 0:
    raise FileNotFoundError('There is no sbm preset (.json)')
  return sbm_preset_filenames[-1]

# depth estimation
def save_poly_linear_model(model, filename = 'depth_estimation_model.pickle'):
  joblib.dump(model, os.path.join(project_dir, default_preset_dir, filename))

def load_poly_linear_model(filename = None):
  if filename is None:
    filename = os.path.join(project_dir, default_preset_dir, 'depth_estimation_model.pickle')
  return joblib.load(filename)

# depth calibration
def save_depth_map(depth_map, filename='depth_map.npy'):
  file_path = os.path.join(project_dir, default_preset_dir, filename)
  # joblib.dump(depth_map, file_path)
  np.save(file_path, depth_map)

def load_depth_map(filepath=None):
  if filepath == None:
    filepath = os.path.join(project_dir, default_preset_dir, 'depth_map.npy')
  # return joblib.load(filepath)
  return np.load(filepath)