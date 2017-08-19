import cv2
import numpy as np
import glob
import math
import pointsCollect
from pythonzenity import Entry


def attitude():
    '''scale factor wich will be also entered by the user'''

    scale=float(Entry(text="what is scale factor", entry_text="1.0"))
    ''''''


    '''getting cameras parameters'''

    with np.load('./camera_coeffs.npz') as X:
	    mtx, dist, _, _ = [X[i] for i in ('arr_0','arr_1','arr_2','arr_3')]

    #print('camera matrix:',mtx)
    #print('distortion coefficients:', dist)
    ''''''


    '''image loading'''

    fname = './image/HD_18_football_3.JPG'
    #fname = './single/HD_18_im6.jpg'
    img = cv2.imread(fname)
    ''''''



    '''choosing the points and defining their coords in 3D'''
    coords_uv,coords_xyz= pointsCollect.get_coords(img)
    uv=np.array(coords_uv, dtype=np.float)
    xyz=np.array(coords_xyz, dtype=np.float)

    # znormalizowany kwadrat
    #xyz=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]],dtype=np.float)
    ''''''



    '''external camera parameters estimation'''
    ret, rvecs, tvecs= cv2.solvePnP(xyz, uv, mtx, dist)

    print('rotation:', rvecs)
    print('translation:', tvecs*scale)
    ''''''
    return [rvecs, tvecs*scale]




