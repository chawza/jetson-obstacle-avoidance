import os
import numpy as np 
import cv2
import calibration
from depthestimation import DepthEstimator
from capture_cam import StereoCams

CamL_id = 0 # Camera ID for left camera
CamR_id = 2 # Camera ID for right camera

cam = StereoCams(CamL_id, CamR_id)

odd = []
for value in np.arange(3, 100):
	if value % 2 == 1:
		odd.append(value)

# Reading the mapping values for stereo image rectification
# preset_path = os.path.join(os.getcwd(), 'notebooks', 'samples', 'stereo_preset_01')
preset_path = './calibration_preset'
cam_preset = calibration.load_calibrate_map_preset(preset_dir=preset_path)

def nothing(x):
    pass

cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',600,600)

estimator = DepthEstimator(cam_preset=cam_preset)
stereo = estimator.stereo

cv2.createTrackbar('numDisparities','disp',1,17,nothing)
cv2.createTrackbar('minDisparity','disp',5,25,nothing)
cv2.createTrackbar('blockSize','disp',5,50,nothing)
cv2.createTrackbar('preFilterType','disp',1,1,nothing)
cv2.createTrackbar('preFilterSize','disp',2,25,nothing)
cv2.createTrackbar('preFilterCap','disp',5,62,nothing)
cv2.createTrackbar('textureThreshold','disp',10,100,nothing)
cv2.createTrackbar('uniquenessRatio','disp',15,100,nothing)
cv2.createTrackbar('speckleRange','disp',0,100,nothing)
cv2.createTrackbar('speckleWindowSize','disp',3,25,nothing)
cv2.createTrackbar('disp12MaxDiff','disp',5,25,nothing)

print('Quit\t\t: Q')
print('Save Preset\t: S')

for imgL, imgR in cam.read():
	numDisparities = cv2.getTrackbarPos('numDisparities','disp')*16 or 16
	minDisparity = cv2.getTrackbarPos('minDisparity','disp')
	blockSize = cv2.getTrackbarPos('blockSize','disp')
	preFilterType = cv2.getTrackbarPos('preFilterType','disp')
	preFilterSize = cv2.getTrackbarPos('preFilterSize','disp')
	preFilterCap = cv2.getTrackbarPos('preFilterCap','disp') or 1
	textureThreshold = cv2.getTrackbarPos('textureThreshold','disp')
	uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio','disp')
	speckleRange = cv2.getTrackbarPos('speckleRange','disp')
	speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize','disp')*2
	disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff','disp')

	preFilterSizeValue = odd[preFilterSize]
	if preFilterSizeValue < 5:
		preFilterSizeValue = 5
	blockSizeValue = odd[blockSize]
	
	# Setting the updated parameters before computing disparity map
	stereo.setNumDisparities(numDisparities)
	stereo.setBlockSize(blockSizeValue)
	stereo.setMinDisparity(minDisparity)
	stereo.setPreFilterType(preFilterType)
	stereo.setPreFilterSize(preFilterSizeValue)
	stereo.setPreFilterCap(preFilterCap)
	stereo.setTextureThreshold(textureThreshold)
	stereo.setUniquenessRatio(uniquenessRatio)
	stereo.setSpeckleRange(speckleRange)
	stereo.setSpeckleWindowSize(speckleWindowSize)
	stereo.setDisp12MaxDiff(disp12MaxDiff)

	disparity = estimator.get_disparity(imgL, imgR)
	disparity = cv2.blur(disparity, (3,3))
	disparity = estimator.normalize_disparity(disparity)
	# NOTE: Code returns a 16bit signed single channel image,
	# CV_16S containing a disparity map scaled by 16. Hence it 
	# is essential to convert it to CV_32F and scale it down 16 times.
	
	cv2.putText(disparity, f'numDisp: {numDisparities}', org=(50, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, color=(255,0,0), thickness= 1, lineType= cv2.LINE_AA)
	cv2.putText(disparity, f'minDisp: {minDisparity}', (50, 75), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, color=(255,0,0), thickness= 1, lineType= cv2.LINE_AA)
	cv2.putText(disparity, f'blockSize: {blockSize}', (50, 100), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale= 1, color=(255,0,0), thickness= 1, lineType= cv2.LINE_AA)
	
	cv2.imshow('Disparity', disparity)
	cv2.imshow('Left Camera', imgL)

	# Close window using esc key
	key = cv2.waitKey(1)

	if key == ord('q'):
		print('numDisparities', numDisparities) # 64
		print('blockSize', blockSizeValue) # 9
		print('minDisparity', minDisparity) # 4
		print('preFilterType', preFilterType)
		print('preFilterSize', preFilterSizeValue)
		print('preFilterCap', preFilterCap)
		print('textureThreshold', textureThreshold)
		print('uniquenessRatio', uniquenessRatio)
		print('speckleRange', speckleRange)
		print('speckleWindowSize', speckleWindowSize)
		print('disp12MaxDiff', disp12MaxDiff)

		cam.clean_up()
		break
	
	if key == ord('s'):
		estimator.save_current_preset()
