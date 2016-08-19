from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
#from subprocess import call, Popen, check_output
import time, math, sys, os, re
import argparse
from pymavlink import mavutil
import subprocess
from pymavlink import fgFDM
import util
 

from subprocess import *

from sandbox import *
from tmpglob import *

dkargs = [ 'dronekit-sitl',
           'copter',  
           '--home=-7.162675,-34.817705,36,250' ]

dkargs2 = [ 'dronekit-sitl',
           'copter',
           '--instance=1',
           '--home=-7.162675,-34.817705,36,250' ]

dkargs3 = [ 'dronekit-sitl',
           'copter',
           '--speedup=100',
           '--rate=10',
           '--home=-7.162675,-34.817705,36,250' ]

mpargs_map = [ 'mavproxy.py',
           '--master',
           'tcp:127.0.0.1:5760',
           '--sitl=127.0.0.1:5501',
           '--out=127.0.0.1:14550',
           '--out=127.0.0.1:14551',
           '--map',
           '--console',
           '--aircraft',
           'test' ]

mpargs_map2 = [ 'mavproxy.py',
           '--master',
           'tcp:127.0.0.1:5770',
           '--sitl=127.0.0.1:5511',
           '--out=127.0.0.1:14560',
           '--out=127.0.0.1:14561',
           '--map',
           '--console',
           '--aircraft',
           'test' ]


mpargs = [ 'mavproxy.py',
           '--master',
           'tcp:127.0.0.1:5760',
           '--sitl=127.0.0.1:5501',
           '--out=127.0.0.1:14550',
           '--out=127.0.0.1:14551',
           '--aircraft',
           'test' ]

class SimpleDrone(object):

    def __init__(self, name):
        """Creates a drone with given name and defautl state.
        """
        self.name = name
        self.vehicle = None
        self.vehicle2 = None
        #self.windParam = WindParam("newWind")
        self.altitude = 5

        self.dronekit = None
        self.mavproxy = None

        self.dronekit2 = None
        self.mavproxy2 = None

        
    def initialize(self):

        self.spawn()
        self.spawn2()

        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)
        self.vehicle2 = connect('127.0.0.1:14560', wait_ready=True)


        return True

    def spawn(self):
        sys.stderr.write("Spawning dronekit\n")
        self.dronekit = SandBox('dronekit', dkargs, False)
        self.dronekit.start()
        sys.stderr.write("dronekit with pid {0} spawned\n".format(self.dronekit.getpid()))

        sys.stderr.write("sleeping\n")
        time.sleep(2)
        sys.stderr.write("slept\n")

        
        sys.stderr.write("Spawning mavproxy\n")
        self.mavproxy =  SandBox('mavproxy', mpargs_map, True)
        self.mavproxy.start()
        #sys.stderr.write("mavproxy with pid {0} spawned\n".format(self.mavproxy.getpid()))

    def spawn2(self):
        sys.stderr.write("Spawning dronekit\n")
        self.dronekit2 = SandBox('dronekit', dkargs2, False)
        self.dronekit2.start()
        sys.stderr.write("dronekit with pid {0} spawned\n".format(self.dronekit2.getpid()))

        sys.stderr.write("sleeping\n")
        time.sleep(2)
        sys.stderr.write("slept\n")

        
        sys.stderr.write("Spawning mavproxy\n")
        self.mavproxy2 =  SandBox('mavproxy', mpargs_map2, True)
        self.mavproxy2.start()
        #sys.stderr.write("mavproxy with pid {0} spawned\n".format(self.mavproxy.getpid()))


    def exit(self):

        if self.mavproxy is not None:
            self.mavproxy.stop()
            sys.stderr.write("mavproxy with pid {0} killed\n".format(self.mavproxy.getpid()))
            self.mavproxy = None

        if self.dronekit is not None:
            self.dronekit.stop()
            sys.stderr.write("dronekit with pid {0} killed\n".format(self.dronekit.getpid()))
            self.dronekit = None
        
        if self.mavproxy2 is not None:
            self.mavproxy2.stop()
            sys.stderr.write("mavproxy with pid {0} killed\n".format(self.mavproxy2.getpid()))
            self.mavproxy2 = None

        if self.dronekit2 is not None:
            self.dronekit2.stop()
            sys.stderr.write("dronekit with pid {0} killed\n".format(self.dronekit2.getpid()))
            self.dronekit2 = None

    def takeOff(self,vh,altitude):
        self.altitude = altitude
        while not vh.is_armable:
           time.sleep(1)
        vh.mode = VehicleMode("GUIDED")
        vh.armed = True
        while not vh.armed:
            time.sleep(1)

        vh.simple_takeoff(self.altitude)
 
    def mv(self, vh, x, y, z, v):
        # while True:
        #     print " Altitude: ", self.vehicle.location.global_relative_frame.alt
        #     if self.vehicle.location.global_relative_frame.alt >= self.altitude*0.95:
        #         print "Reached target altitude"
        #         break
        #     time.sleep(1)

        currentLocation = vh.location.global_relative_frame
        sys.stderr.write('Current: {0}'.format(currentLocation))
        #print "Current: ", currentLocation
        targetLocation = get_location_metres(currentLocation, float(y), float(x))
        sys.stderr.write('Target: {0}'.format(targetLocation))
        #print "Target: ", targetLocation
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        sys.stderr.write('Target Distance: {0}'.format(targetDistance))
        #print "Target Distance: ", targetDistance
        # gotoFunction(targetLocation)

##      self.vehicle.airspeed=float(10)
        vh.airspeed=float(v)
        # self.vehicle.simple_goto(targetLocation,groundspeed=10)
        vh.simple_goto(targetLocation)

        # point = LocationGlobalRelative(float(x),float(y),float(z))
        # self.vehicle.simple_goto(point)
        return True

    def charge(self, amt):
        self.e += float(amt)
        return True

    def land(self,vh):
        self.send_global_velocity(vh,0,0,0,1)
        vh.mode = VehicleMode("LAND")

    def send_global_velocity(self,vh,velocity_x, velocity_y, velocity_z, duration):
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
        msg = vh.message_factory.set_position_target_global_int_encode(
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
            vh.send_mavlink(msg)
            time.sleep(1)


    def __str__(self):
        if self.vehicle is not None:
            pos =  re.findall('[-+]?\d+[\.]?\d*', str(self.vehicle.location.local_frame))
            if not pos:
                pos = [0, 0, 0]
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
        else:
            return 'Uninitialized'
                
    def vivek__str__(self):
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

"""
from sitl_drone_tests import *
      
x = SimpleDrone("hello")
x.initialize()

x.takeOff(x.vehicle,5)        
#"Go to Destination 1"
x.mv(x.vehicle,100,0,5,3)

x.takeOff(x.vehicle2,5)
x.mv(x.vehicle2,100,0,5,3)

#"Start Position: "
x.vehicle.location.local_frame


x.exit()

# cuurent location
str(x.vehicle.location.local_frame)

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


"""


