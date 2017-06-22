import cv2
import numpy as np



def get_coords(img):
    coords=[]
    
    
    # mouse callback function
    def get_one(event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(img,(x,y),3,(0,0,255),-1)
            i_coord=(x,y)
            coords.append(i_coord)
    
    
    cv2.namedWindow('image')
    cv2.setMouseCallback('image',get_one)
    while(1):
        cv2.imshow('image',img)
        if cv2.waitKey(20) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    return coords



#img = np.zeros((512,512,3), np.uint8)
#print get_coords(img)