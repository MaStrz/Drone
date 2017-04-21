import cv2
import numpy as np
import glob
import math



with np.load('./output/camera_coeffs.npz') as X:
	mtx, dist, _, _ = [X[i] for i in ('arr_0','arr_1','arr_2','arr_3')]

#print('camera matrix:',mtx)
#print('distortion coefficients:', dist)




# function for drawing 3D Axis 
def draw(img, corners, imgpts):
	corner = tuple(corners[0].ravel())
	img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
	img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
	img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
	return img


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)


# Loading of image

fname = './single/HD_18_archi.JPG'
#fname = './single/HD_18_im6.jpg'

img = cv2.imread(fname)

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, corners = cv2.findChessboardCorners(gray, (9,6),None)


if ret == True:
	corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

        # Find the rotation and translation vectors.
	ret, rvecs, tvecs= cv2.solvePnP(objp, corners2, mtx, dist)

	print('rotation:', rvecs)
	print('translation:', tvecs*26)

        # project 3D points to image plane
	imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
	img = draw(img,corners2,imgpts)
	cv2.imshow('img',img)
	cv2.imwrite('./output/localAxis.png', img)
	k = cv2.waitKey(0) & 0xFF
	if k == ord('s'):
		cv2.imwrite(fname[:6]+'.png', img)
'''

# Find the rotation and translation vectors.
corners2 = cv2.cornerSubPix(gray,corners,(26,26),(-1,-1),criteria)
ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
print('translation:', tvecs)
'''
cv2.destroyAllWindows()





# Trying of desired camera coords computing

pi=3.14
A=rvecs[0]
B=rvecs[1]
C=rvecs[2]



# Rotation matrix from rotation vect.

R = np.array([[ math.cos(C)*math.cos(B), 	-math.sin(C)*math.cos(A)+math.cos(C)*math.sin(B)*math.sin(A) , 		math.sin(C)*math.sin(A) + math.cos(C)*math.sin(B)*math.cos(A)], 
                  [ math.sin(C)*math.cos(B) , 	math.cos(C)*math.cos(A)+math.sin(C)*math.sin(B)*math.sin(A)  , 	-math.cos(C)*math.sin(A)+math.sin(C)*math.sin(B)*math.cos(A)], 
                  [ -math.sin(B),  	math.cos(B)*math.sin(A) , 	math.cos(B)*math.cos(A)]])


R = np.transpose(R)
b = np.array([52, 52, 104])

coords = b.dot(R)
coords[0] += tvecs[0]
coords[1] += tvecs[1]
coords[2] += tvecs[2]

print coords




