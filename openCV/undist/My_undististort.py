import cv2
import numpy as np

fname = './2016_0319_215501_003.JPG'
img = cv2.imread(fname)
      

with np.load('./camera_coeffs.npz') as X:
	mtx, dist, _, _ = [X[i] for i in ('arr_0','arr_1','arr_2','arr_3')]















dst = cv2.undistort(img, mtx, dist, None)

cv2.imwrite('calibresult.png',dst)

