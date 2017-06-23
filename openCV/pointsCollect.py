import cv2
import numpy as np
from pythonzenity import Entry


def get_coords(img):
    coords_2d=[]
    coords_3d=[]
    
    
    # mouse callback function
    def get_one(event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(img,(x,y),5,(0,0,255),-1)
            co_2d=(x,y)
            coords_2d.append(co_2d)
            
            x3d=float(Entry(text="write X coord in 3D", entry_text="0.0"))
            y3d=float(Entry(text="write Y coord in 3D", entry_text="0.0"))
            z3d=float(Entry(text="write Z coord in 3D", entry_text="0.0"))
            
            co_3d=(x3d,y3d,z3d)
            coords_3d.append(co_3d)
            

    
    
    cv2.namedWindow('image')
    cv2.setMouseCallback('image',get_one)
    while(1):
        cv2.imshow('image',img)
        if cv2.waitKey(20) & 0xFF == 32:
            break
    cv2.imwrite('./output/PnP.png', img)
    cv2.destroyAllWindows()
    
    return coords_2d, coords_3d



#img = np.zeros((512,512,3), np.uint8)
#print get_coords(img)