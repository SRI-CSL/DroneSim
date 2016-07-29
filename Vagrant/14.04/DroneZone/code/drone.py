from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

import time, math, sys, os
import argparse
from pymavlink import mavutil

from subprocess import *

from code.sandbox import *


dkargs = [ 'dronekit-sitl',
           'copter',
           '--home=-7.162675,-34.817705,36,250' ]

mpargs = [ 'mavproxy.py',
           '--master',
           'tcp:127.0.0.1:5760',
           '--sitl=127.0.0.1:5501',
           '--out=127.0.0.1:14550',
           '--out=127.0.0.1:14551',
           '--map',
           '--console',
           '--aircraft',
           'test' ]


class SimpleDrone(object):
    """The simplest drone that can be managed by the python Actor.
    """

    def __init__(self, name):
        """Creates a drone with given name and defautl state.
        """
        self.name = name
        self.x = 0
        self.y = 0
        self.z = 0
        self.v = 0
        self.e = 10.0

        self.dronekit = None
        self.mavproxy = None
        self.vehicle = None

        
        self.altitude = 100


    def spawn(self):
        sys.stderr.write("Spawning dronekit\n")
        self.dronekit = SandBox('dronekit', dk_argv, False)
        self.dronekit.start()
        sys.stderr.write("dronekit with pid {0} spawned\n".format(self.dronekit.getpid()))

        sys.stderr.write("sleeping\n")
        time.sleep(2)
        sys.stderr.write("slept\n")

        
        sys.stderr.write("Spawning mavproxy\n")
        self.mavproxy =  SandBox('mavproxy', mp_argv, True)
        self.mavproxy.start()
        sys.stderr.write("mavproxy with pid {0} spawned\n".format(self.mavproxy.getpid()))

        
    def initialize(self, x, y, z, v, e):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.v = float(v)
        self.e = float(e)

        self.spawn()
        
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)

        
        return True


    def exit(self):

        if self.mavproxy is not None:
            self.mavproxy.stop()
            sys.stderr.write("mavproxy with pid {0} killed\n".format(self.mavproxy.pid))
            self.mavproxy = None

        if self.dronekit is not None:
            self.dronekit.stop()
            sys.stderr.write("dronekit with pid {0} killed\n".format(self.dronekit.pid))
            self.dronekit = None


    
    def mv(self, x, y, z, v):

        while not self.vehicle.is_armable:
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

    	self.vehicle.simple_takeoff(self.altitude)

    	while True:
    		if self.vehicle.location.global_relative_frame.alt >= self.altitude*0.95:
    			break
    		time.sleep(1)
        self.vehicle.airspeed=v
        point = LocationGlobalRelative(x,y,z)
        self.vehicle.simple_goto(point)
        return True

    def charge(self, amt):
        self.e += float(amt)
        return True


    def __str__(self):
        pos = extractNumber(str(vehicle.location.local_frame))
        return '{0} {1} {2} {3} {4}'.format(pos[0], pos[1], pos[2], vehicle.velocity, vehicle.battery.level)


#from code.drone import *
#x = SimpleDrone("hello")
#x.initialize(-7.162675,-34.817705,36,10,10)
#x.mv(-7.163147,-34.818550,2,23)
