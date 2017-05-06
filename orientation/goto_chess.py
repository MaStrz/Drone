#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math






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


#vehicle.LocationLocal(0,0,0)
pose=vehicle.location.local_frame
print ("local location: %s" % pose)
print pose

print("NED movement   [10, 10, -10] ")
goto_ned(vehicle.location.local_frame.north-10, vehicle.location.local_frame.east+10, vehicle.location.local_frame.down)
#print("local mov [20,20,-10]")
#a_location = LocationGlobalRelative(0, 0, -10)
#vehicle.simple_goto(a_location)



#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")















