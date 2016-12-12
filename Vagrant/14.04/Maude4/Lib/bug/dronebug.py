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

class Drone(object):
    def __init__(self, name, instance_no=0, debug=False, speedup=None):
        """Creates a drone with given name and default state.
        """
        self.name = name
        self.debug = debug
        self.ino = instance_no
        self.pipeincr = 10 * self.ino
        self.speedup = speedup

        if self.debug:
            sys.stderr.write("SimpleDrone with name {0} and instance number {1}\n".format(self.name, self.ino))

        self.x = 0
        self.y = 0
        self.z = 0
        self.v = 0
        self.e = 10.0
        self.altitude = 5

        self.vehicle = None

        self.dronekit = None
        self.mavproxy = None

    def initialize(self):

        self.spawn()

        self.vehicle = connect(self.sitl_ip() , wait_ready=True)

        return True

    def sitl_ip(self):
        return '127.0.0.1:{0}'.format(14550 + self.pipeincr)

    def drokekit_args(self):
        dkargs = [ 'dronekit-sitl',
                   'copter-3.3',
                   '--instance',
                   '{0}'.format(self.ino),
                   '--home=-7.162675,-34.817705,36,250' ]
        if self.speedup is not None:
            dkargs.append('--speedup={0}'.format(self.speedup))
        return dkargs

    def mavproxy_args(self):
        mpargs = [ 'mavproxy.py',
                   '--master',
                   'tcp:127.0.0.1:{0}'.format(5760 + self.pipeincr),
                   '--sitl=127.0.0.1:{0}'.format(5501 + self.pipeincr),
                   '--out=127.0.0.1:{0}'.format(14550 + self.pipeincr),
                   '--aircraft',
                   '/tmp/drone_{0}'.format(self.ino) ]
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
        self.vehicle.mode = VehicleMode("GUIDED")
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

"""
from dronebug import *

x = Drone("hello", debug=True, speedup=3)
x.initialize()
x.takeOff(5)
x.mv(20,20,5,3)


"""
