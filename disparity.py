import numpy as np

class StereoMatcher():
  def __init__(self, window_size = 3, treshold = 0):
    self.window_size = window_size
    self.treshold = treshold

  def compute(self, left, right):
    disparity = np.ndarray(left.shape, dtype=np.dtype('uint8'))
    row_iter = int(left.shape[0] / self.window_size)
    col_iter = int(left.shape[1] / self.window_size)

    # fill right most area that will not get compared
    # disparity[col_iter * self.window_size: , :] = np.float64(0)

    # Memoization variable to hold Sum Absolute of img area
    self.left_sum = np.ndarray((col_iter, row_iter), dtype=np.dtype('uint8'))
    self.right_sum = np.ndarray((col_iter, row_iter), dtype=np.dtype('uint8'))

    # Iterate all row scanline
    for row in range(row_iter):
      y_start = row * self.window_size
      y_end = y_start + self.window_size

      for col_x in range(col_iter):
        x_start = col_x * self.window_size
        x_end = x_start + self.window_size

        target = right[x_start: x_end, y_start: y_end]
        self.right_sum[col_x, row] = np.sum(target)
      
      # scan current pattern to current scanline
      for col in range(col_iter):
        x_start = col * self.window_size
        x_end = x_start + self.window_size

        # calculate sum absoule in current patch
        pattern = left[x_start : x_end, y_start: y_end]
        self.left_sum[col, row] = np.sum(pattern)

        # calculate sum absoulte of all target patches in scanline
        
        # get all SAD difference in each row
        differences = []
        for scan_idx in range(col_iter):
          # calculate difference
          differences.append(self.left_sum[col, row] - self.right_sum[scan_idx, row])
        
        # calculate disparity
        scanline_idx = np.argmin(differences)
        match_target = (col + scanline_idx) * self.window_size
        disparity[x_start : x_end, y_start: y_end] = x_start - match_target

    return disparity

    





