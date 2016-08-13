from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
#from subprocess import call, Popen, check_output
import time, math, sys, os, re
import argparse
from pymavlink import mavutil
import subprocess
from pymavlink import fgFDM
import util

# (define b0 (apply mkdrone "b0" "0" "0" "0" "14" "15"))
# (invoke b0 "mv" "-7.163147" "-34.817705" "2" "23")
# (invoke b0 "mv" "7.163147" "-3.818550" "2" "23")
# b0
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

class WindParam(object):

    def __init__(self,name):

        self.name = name
        self.speed = 0.0
        self.direction = 0.0
        self.turbulance = 0.0


class SimpleDrone(object):

    def __init__(self, name):
        """Creates a drone with given name and defautl state.
        """
        self.name = name
        self.x = 0
        self.y = 0
        self.z = 0
        self.v = 0
        self.e = 10.0
        self.vehicle = None
        self.windParam = WindParam("newWind")
        self.altitude = 5
        
    def initialize(self, windstr):

        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)

        self.vehicle.wind = util.Wind(windstr)
        while not self.vehicle.is_armable:
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

        self.vehicle.simple_takeoff(self.altitude)

        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt
            if self.vehicle.location.global_relative_frame.alt >= self.altitude*0.95:
                print "Reached target altitude"
                break
            time.sleep(1)

        return True

    def mv(self, x, y, z, v):
        # while True:
        #     print " Altitude: ", self.vehicle.location.global_relative_frame.alt
        #     if self.vehicle.location.global_relative_frame.alt >= self.altitude*0.95:
        #         print "Reached target altitude"
        #         break
        #     time.sleep(1)

        currentLocation = self.vehicle.location.global_relative_frame
        print "Current: ", currentLocation
        targetLocation = get_location_metres(currentLocation, float(y), float(x))
        print "Target: ", targetLocation
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        print "Target Distance: ", targetDistance
        # gotoFunction(targetLocation)

##      self.vehicle.airspeed=float(10)
        self.vehicle.airspeed=float(v)
        # self.vehicle.simple_goto(targetLocation,groundspeed=10)
        self.vehicle.simple_goto(targetLocation)

        # point = LocationGlobalRelative(float(x),float(y),float(z))
        # self.vehicle.simple_goto(point)
        return True

    def charge(self, amt):
        self.e += float(amt)
        return True

    def land(self):
        self.send_global_velocity(0,0,0,1)
        self.vehicle.mode = VehicleMode("LAND")

    def send_global_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
        velocity components 
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
        
        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version 
        (sending the message multiple times does not cause problems).
        
        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def __str__(self):
        pos =  re.findall('[-+]?\d+[\.]?\d*', str(self.vehicle.location.local_frame))
        auxVel = self.vehicle.velocity
        bat = self.vehicle.battery.level
        dx = auxVel[0]
        dy = auxVel[1]
        dz = auxVel[2]
        vel = math.sqrt(math.pow(dx,2) + math.pow(dy,2) + math.pow(dz,2))
        dx = dx / vel
        dy = dy / vel
        dz = dz / vel
        return '{0} {1} {2} {3} {4} {5} {6} {7}'.format(pos[0], pos[1], pos[2], dx, dy, dz, vel, bat)

x = SimpleDrone("hello")
x.initialize("0,0,0")
# x.initialize("10,1.57,0")
print "Start Position: ", x.vehicle.location.local_frame
print "Go to Destination 1"
x.mv(-1000,0,10,3)
time.sleep(100)
print "Destination 1 :", x.vehicle.location.local_frame
# print "Go to Destination 2"
# x.mv(0,100,10,10)
# time.sleep(15)
# print "Destination 2"
# print x.vehicle.location.local_frame
# print "Go to Destination 3"
# x.mv(-100,0,10,10)
# time.sleep(15)
# print "Destination 3"
# print x.vehicle.location.local_frame
# print "Go to Destination 4"
# x.mv(0,-100,10,10)
# time.sleep(15)
# print "Destination 4"
# print x.vehicle.location.local_frame
print "Landing"
x.land()
time.sleep(10)
print "Landed at: ", x.vehicle.location.local_frame
print "Battery: ", x.vehicle.battery.level

# (latH,lonH) --> (0,0)
# 1 unit in my grid = d meters

# (lat,lon) --> (x,y)
# y = get_distance_metres(latH,X,lat,X) / d
# x = get_distance_metres(X,lonH,X,latH) / d

# Converting grid to lat and lon
# (x,y) --> (lat,lon)
# (lat,lon) = get_location_metres(latH,lonH,y,x)




