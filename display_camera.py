import cv2

# 0 == LEFT
# 1 == Acer CAM
# 2 == Right

left_cam_idx = 0
right_cam_idx = 1

cam_left = cv2.VideoCapture(left_cam_idx)
cam_right = cv2.VideoCapture(right_cam_idx)

if not cam_left.isOpened():
  raise Exception('Cannot read Left Camera')
if not cam_right.isOpened():
  raise Exception('Cannot read Right Camera')

while True:
  left_is_captured, left_img = cam_left.read()
  right_is_captured, right_img = cam_right.read()

  cv2.imshow(f'Left Camera', left_img)
  cv2.imshow(f'Right Camera', right_img)  

  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

cam_left.release()
cam_right.release()
cv2.destroyAllWindows()
