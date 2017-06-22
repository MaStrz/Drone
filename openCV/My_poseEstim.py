import cv2
import numpy as np
import glob
import math
import pointsCollect



with np.load('./output/camera_coeffs.npz') as X:
	mtx, dist, _, _ = [X[i] for i in ('arr_0','arr_1','arr_2','arr_3')]

print('camera matrix:',mtx)
print('distortion coefficients:', dist)






# Loading of image
fname = './single/HD_18_archi.JPG'
#fname = './single/HD_18_im6.jpg'
img = cv2.imread(fname)

coords= pointsCollect.get_coords(img)
uv=np.array(coords, dtype=np.float)
print('uv',uv)

#uv=np.array([[[1058.742675, 323.85943604]],[[1152.22558594,   330.33380127]],[[1243.70117188,   338.4213562]])

# znormalizowany kwadrat
xyz=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]],dtype=np.float)



ret, rvecs, tvecs= cv2.solvePnP(xyz, uv, mtx, dist)

print('rotation:', rvecs)
print('translation:', tvecs*78)

        

