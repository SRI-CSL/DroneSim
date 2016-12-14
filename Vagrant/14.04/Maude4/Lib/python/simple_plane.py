from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time, math, sys, os, re
import argparse
from pymavlink import mavutil
import subprocess
from pymavlink import fgFDM
import util


from subprocess import *

from sandbox import *
from tmpglob import *


"""
At some stage we want to pass in the position, altitude and yaw to
the constructor. But currently we are hardcode with

7.162675, -34.817705, 36,      250
latitude  longitude   altitude yaw



"""

class SimplePlane(object):
    """
    plambda lesson:

    after

    (import "simple_plane")

    can make a drone like so:

    (apply simple_plane.SimplePlane  "drone_0")

    and get the defaults for instance_no, debug, and speedup.

    or:

    (let ((largs (mklist "drone_0"))
          (dargs (mkdict "instance_no" (int 666) "debug" (boolean False) "speedup" (int 6))))
       (kwapply simple_drone.SimplePlane largs dargs))

    and get a customized drone whose battery will run out quicker.
    kw stands for keyword.

    """

    def __init__(self, name, instance_no=0, debug=True, speedup=3):
        """Creates a drone with given name and default state.
        """
        self.name = name
        self.debug = debug
        self.ino = instance_no
        self.pipeincr = 10 * self.ino
        self.speedup = speedup

        if self.debug:
            sys.stderr.write("SimplePlane with name {0} and instance number {1}\n".format(self.name, self.ino))

        self.x = 0
        self.y = 0
        self.z = 0
        self.v = 0
        self.e = 10.0
        self.altitude = 5

        self.vehicle = None

        self.dronekit = None
        self.mavproxy = None

    def getName(self):
        return self.name

    def initialize(self):

        self.spawn()

        self.vehicle = connect(self.sitl_ip() , wait_ready=True)

        self.vehicle.wait_ready('autopilot_version')

        print 'heading: ', self.vehicle.heading

        return True

    def sitl_ip(self):
        return '127.0.0.1:{0}'.format(14550 + self.pipeincr)

    def drokekit_args(self):
        return [ 'dronekit-sitl',
                 'plane-3.3.0',
                 '--instance',
                 '{0}'.format(self.ino),
                 '--speedup={0}'.format(self.speedup),
                 '--home=-7.162675,-34.817705,36,250' ]

    def mavproxy_args(self):
        mpargs = [ 'mavproxy.py',
                   '--master',
                   'tcp:127.0.0.1:{0}'.format(5760 + self.pipeincr),
                   '--sitl=127.0.0.1:{0}'.format(5501 + self.pipeincr),
                   '--out=127.0.0.1:{0}'.format(14550 + self.pipeincr),
                   '--aircraft',
                   '/tmp/plane_{0}'.format(self.ino) ]
        if self.debug:
            mpargs.extend(['--map', '--console'])
        return mpargs


    def spawn(self):
        dkargs = self.drokekit_args()
        if self.debug:
            sys.stderr.write("Spawning dronekit {0}\n".format(dkargs))
        self.dronekit = SandBox('dronekit', dkargs, False)
        self.dronekit.start()
        if self.debug:
            sys.stderr.write("dronekit with pid {0} spawned\n".format(self.dronekit.getpid()))
            sys.stderr.write("sleeping\n")

        time.sleep(2)
        if self.debug:
            sys.stderr.write("slept\n")

        mpargs = self.mavproxy_args()

        if self.debug:
            sys.stderr.write("Spawning mavproxy {0}\n".format(mpargs))
        self.mavproxy =  SandBox('mavproxy', mpargs, True)
        self.mavproxy.start()
        if self.debug:
            sys.stderr.write("mavproxy with pid {0} spawned\n".format(self.mavproxy.getpid()))

    def takeOff(self, altitude):
        self.altitude = float(altitude)
        while not self.vehicle.is_armable:
           time.sleep(1)
        self.vehicle.mode = VehicleMode("AUTO")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

        self.vehicle.simple_takeoff(self.altitude)

    def mv(self, x, y, z, v):
        currentLocation = self.vehicle.location.global_relative_frame
        if self.debug:
            sys.stderr.write('Current: {0}\n'.format(currentLocation))
        targetLocation = get_location_metres(currentLocation, float(y), float(x))
        if self.debug:
            sys.stderr.write('Target: {0}\n'.format(targetLocation))
        targetDistance = get_distance_metres(currentLocation, targetLocation)
        if self.debug:
            sys.stderr.write('Target Distance: {0}\n'.format(targetDistance))
        self.vehicle.airspeed=float(v)
        self.vehicle.simple_goto(targetLocation)
        return True

    def charge(self, amt):
        self.e += float(amt)
        return True

    def land(self):
        #self.send_global_velocity(0,0,0,1)
        # http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
        self.vehicle.groundspeed=0
        self.vehicle.mode = VehicleMode("LAND")
        return True

    def stop(self):
        return True

    def rtl(self):
        self.vehicle.mode = VehicleMode("RTL")

    def reset(self):
        self.shutdown()
        self.initialize()
        return True

    def shutdown(self):
        if self.vehicle is not None:
            self.vehicle.close()
            self.vehicle = None
        if self.mavproxy is not None:
            self.mavproxy.stop()
            if self.debug:
                sys.stderr.write("mavproxy with pid {0} killed\n".format(self.mavproxy.getpid()))
            self.mavproxy = None
        if self.dronekit is not None:
            self.dronekit.stop()
            if self.debug:
                sys.stderr.write("dronekit with pid {0} killed\n".format(self.dronekit.getpid()))
            self.dronekit = None
        # sys.stderr.write('STOP STOP STOP STOP \n STOP STOP STOP STOP \n STOP STOP STOP STOP \n STOP STOP STOP STOP \n STOP STOP STOP STOP \nSTOP STOP STOP STOP \n')

    def log(self):
        if self.vehicle is not None and self.vehicle.location.local_frame.north is not None:
            north =  self.vehicle.location.local_frame.north
            east =  self.vehicle.location.local_frame.east
            alt =  -self.vehicle.location.local_frame.down
            auxVel = self.vehicle.velocity
            bat = self.vehicle.battery.level
            dx = auxVel[0]
            dy = auxVel[1]
            dz = auxVel[2]
            vel = math.sqrt(math.pow(dx,2) + math.pow(dy,2) + math.pow(dz,2))
            if vel != 0:
                dx = dx / vel
                dy = dy / vel
                dz = dz / vel
            return '{0} {1} {2} {3} {4} {5} {6} {7}'.format(east, north, alt, dx, dy, dz, vel, bat)
        else:
            return 'Uninitialized'

    def goToW(self,vx,vy,vz,wx,wy,wz,dur):
         sys.stderr.write("HERE\n")
         # sys.stderr.write('{0} {1} {2} {3} {4} {5} {6}'.format(vx,vy,vz,wx,wy,wz,dur))
         # self.send_ned_velocity(float(vx) + float(wx),float(vy) + float(wy),float(vz) + float(wz),2)
         self.send_global_velocity( (float(vy) + float(wy)),(float(vx) + float(wx)), 0.0,2)
         # self.send_global_velocity(-1.0,1.0,0.0,2)

    def __str__(self):
        if self.vehicle is not None and self.vehicle.location.local_frame.north is not None:
            north =  self.vehicle.location.local_frame.north
            east =  self.vehicle.location.local_frame.east
            alt =  -self.vehicle.location.local_frame.down
            auxVel = self.vehicle.velocity
            bat = self.vehicle.battery.level
            dx = auxVel[0]
            dy = auxVel[1]
            dz = auxVel[2]
            vel = math.sqrt(math.pow(dx,2) + math.pow(dy,2) + math.pow(dz,2))
            if vel != 0:
                dx = dx / vel
                dy = dy / vel
                dz = dz / vel
            return '{0} {1} {2} {3} {4} {5} {6} {7}'.format(east, north, alt, dx, dy, dz, vel, bat)
        else:
            return 'Uninitialized'

    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def send_global_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
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
        # for x in range(0,duration):
        self.vehicle.send_mavlink(msg)
            # time.sleep(1)

"""
x.vehicle.location.local_frame.north
from simple_plane import *

x = SimplePlane("hello")
x.initialize()
x.takeOff(5)

#"Start Position: "
x.vehicle.location.local_frame

#"Go to Destination 1"
x.mv(10,0,5,3)

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
