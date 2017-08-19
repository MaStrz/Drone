#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
import cv2
import numpy as np
import glob
import My_poseEstim
# import photo

pi=3.14

with np.load('./camera_coeffs.npz') as X:
	mtx, dist, _, _ = [X[i] for i in ('arr_0','arr_1','arr_2','arr_3')]



#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None






#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


	
# Connect to the vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)










# function computing pose of desired point in camera's coords

def compute_camCoords(fname):

    # Find the rotation and translation vectors.
    vecs = My_poseEstim.attitude()
    rvecs=vecs[0]
    tvecs=vecs[1]
    print('rotation:', rvecs)
    print('translation:', tvecs)

    # Trying of desired camera coords computing
    A=rvecs[0]
    B=rvecs[1]
    C=-rvecs[2]

    # Rotation matrix from rotation vect.
    R = np.array([[ math.cos(C)*math.cos(B), 	-math.sin(C)*math.cos(A)+math.cos(C)*math.sin(B)*math.sin(A) , 		math.sin(C)*math.sin(A) + math.cos(C)*math.sin(B)*math.cos(A)], 
                  [ math.sin(C)*math.cos(B) , 	math.cos(C)*math.cos(A)+math.sin(C)*math.sin(B)*math.sin(A)  , 	-math.cos(C)*math.sin(A)+math.sin(C)*math.sin(B)*math.cos(A)], 
                  [ -math.sin(B),  	math.cos(B)*math.sin(A) , 	math.cos(B)*math.cos(A)]])


    R = np.transpose(R)
    d = np.array([33.5,20,10])	#33.5, 20, 10

    coords = d.dot(R)
    coords[0] += tvecs[0]   
    coords[1] += tvecs[1]   
    coords[2] += tvecs[2]   

    print ('coords in camera axis',coords)
    return coords











# function computing NED coords from camera's coords 

def compute_ned(camCoords): 

    #camCoords = np.array([x, y, z])
    hdg = vehicle.heading
    print " Heading: %s" % hdg
    teta= hdg*2*pi/360
    print teta

    R= np.array([[np.cos(teta),    0    , np.sin(teta)],
            [    0       ,    1    ,       0     ],
            [ -np.sin(teta),  0    , np.cos(teta)]])

    R = np.transpose(R)
    NEDPose=camCoords.dot(R)
    print ('NED',NEDPose)
    return NEDPose







def goto_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111111000, 
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, 
        0, 0, 0, 
        0, 0)    
    # send command to vehicle
    vehicle.send_mavlink(msg)











pose=vehicle.location.local_frame
print ("local location: %s" % pose)
print pose


NEDPose=compute_ned(compute_camCoords('./image/HD_18_football_3.JPG'))
print "NED movement   %s " % NEDPose
print NEDPose[0]
goto_ned(NEDPose[2], NEDPose[0], vehicle.location.local_frame.down)
#goto_ned(vehicle.location.local_frame.north+NEDPose[2], vehicle.location.local_frame.east+NEDPose[0], vehicle.location.local_frame.down+NEDPose[1],)


#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")















