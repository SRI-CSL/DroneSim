import time
import math
import sys
import threading

import sandbox
import drone_utils
import dronekit_sitl


import dronekit
import pymavlink

tracing = False

binary_path = '/home/vagrant/Repositories/ardupilot/build/sitl/bin/arducopter-quad'

params_path = '/home/vagrant/Repositories/ardupilot/Tools/autotest/default_params/copter.parm'

home_default = '-7.162675,-34.817705,36,250'

init_squark = """
SitlDrone:
\tname:             {0}
\tinstance number:  {1}
\tbinary:           {2}
\tparams:           {3}
\tdebug:            {4}
\tspeedup:          {5}
"""

def sitl_main(drone):
    sitl = drone.sitl
    args = ['--home={0}'.format(drone.home)]
    sitl.launch(args, verbose=drone.debug)
    sitl.block_until_ready(verbose=drone.debug)



class SitlDrone(object):
    """
    plambda lesson:

    after

    (import "sitl_drone")

    can make a drone like so:

    (apply sitl_drone.SitlDrone  "drone_0")

    and get the defaults for instance_no, binary, params, home, debug, and speedup.

    or:

    (let ((largs (mklist "drone_0"))
          (dargs (mkdict "instance_no" (int 666) "debug" (boolean False))))
       (kwapply sitl_drone.SitlDrone largs dargs))

    and get a customized drone whose battery will run out quicker.
    kw stands for keyword.

    """


    def __init__(self, name, instance_no=0, debug=True, speedup=None, binary=binary_path, params=params_path, home=home_default):
        """Creates a drone with given name and default state.
        """
        self.name = name
        self.ino = instance_no
        self.debug = debug
        self.speedup = int(speedup) if speedup is not None else None
        self.binary = binary
        self.params = params
        self.home = home


        self.pipeincr = 10 * self.ino

        #self.sitl = dronekit_sitl.SITL(instance=self.ino, path=binary, defaults_filepath=params)

        if self.debug:
            sys.stderr.write(init_squark.format(self.name, self.ino, self.binary, self.params, self.debug, self.speedup))

        #<iam thinks these are  obsolete>
        self.x = 0
        self.y = 0
        self.z = 0
        self.v = 0
        self.e = 10.0
        self.altitude = 5
        #</iam thinks these are  obsolete>


        self.vehicle = None

        self.mavproxy = None
        self.sitl = None

    def getName(self):
        self.trace("getName")
        return self.name

    def initialize(self):

        self.spawn()

        self.vehicle = dronekit.connect(self.sitl_ip(14550), wait_ready=True)

        self.setSpeedup(self.speedup)

        return True

    def sitl_ip(self, base_port):
        return '127.0.0.1:{0}'.format(base_port + self.pipeincr)

#    def spawn_sitl(self):
#        thread = threading.Thread(target=sitl_main, name='sitl_main_of_{0}'.format(self.name), args=(self, ))
#        #thread needs to be a daemon so that the actor itself can die in peace.
#        thread.daemon = True
#        thread.start()


    def sitl_args(self):
        sargs = ['launch_sitl', str(self.ino), self.binary, self.params, self.home, str(self.debug)]
        return sargs

    def mavproxy_args(self):
        mpargs = ['mavproxy.py',
                  '--master',
                  'tcp:127.0.0.1:{0}'.format(5760 + self.pipeincr),
                  '--sitl={0}'.format(self.sitl_ip(5501)),
                  '--out=127.0.0.1:{0}'.format(14550 + self.pipeincr),
                  '--aircraft',
                  '/tmp/drone_{0}'.format(self.ino)]
        if self.debug:
            mpargs.extend(['--map', '--console'])
        return mpargs


    def trace(self, name):
        if tracing:
            d = self.data()
            f = '{0}.{1} mode = {2} vel = {3} alt = {4}\n'
            if len(d) > 0:
                sys.stderr.write(f.format(self.ino, name, self.vehicle.mode, d['vel'], d['alt']))
            else:
                sys.stderr.write(f.format(self.ino, name, self.vehicle.mode, 'U', 'U'))


    def spawn(self):

        sitl_args = self.sitl_args()

        if self.debug:
            sys.stderr.write('Spawning sitl: {0}\n'.format(sitl_args))

        #self.spawn_sitl()

        self.sitl = sandbox.SandBox('sitl', sitl_args, True)

        self.sitl.start()
        if self.debug:
            sys.stderr.write('sitl with pid {0} spawned\n'.format(self.sitl.getpid()))

        time.sleep(2)
        if self.debug:
            sys.stderr.write('slept\n')

        mpargs = self.mavproxy_args()

        if self.debug:
            sys.stderr.write('Spawning mavproxy: {0}\n'.format(mpargs))
        self.mavproxy = sandbox.SandBox('mavproxy', mpargs, True)
        self.mavproxy.start()
        if self.debug:
            sys.stderr.write('mavproxy with pid {0} spawned\n'.format(self.mavproxy.getpid()))


    def setSpeedup(self, speedup):
        """Set the speedup of the simulation.

        -- speedup the factor by which we speed up.

        """
        self.trace("takeOff")
        self.speedup = int(speedup) if speedup is not None else None
        if self.speedup is not None:
            while True:
                self.vehicle.parameters['SIM_SPEEDUP'] = self.speedup
                sys.stderr.write('Setting SIM_SPEEDUP to {0}\n'.format(self.speedup))
                if self.vehicle.parameters['SIM_SPEEDUP'] == self.speedup:
                    break
                time.sleep(0.1)


    def takeOff(self, altitude):
        self.trace("takeOff")
        self.altitude = float(altitude)
        while not self.vehicle.is_armable:
           time.sleep(1)
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            time.sleep(1)

        self.vehicle.simple_takeoff(self.altitude)

    def mv(self, x, y, z, v):
        self.trace("mv")
        currentLocation = self.vehicle.location.global_relative_frame
        if self.debug:
            sys.stderr.write('Current: {0}\n'.format(currentLocation))
        targetLocation = drone_utils.get_location_metres(currentLocation, float(y), float(x))
        if self.debug:
            sys.stderr.write('Target: {0}\n'.format(targetLocation))
        targetDistance = drone_utils.get_distance_metres(currentLocation, targetLocation)
        if self.debug:
            sys.stderr.write('Target Distance: {0}\n'.format(targetDistance))
        self.vehicle.airspeed = float(v)
        self.vehicle.simple_goto(targetLocation)
        return True

    def charge(self, amt):
        self.trace("charge")
        self.e += float(amt)
        return True

    def land(self):
        self.trace("land")
        # http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
        self.vehicle.groundspeed = 0
        self.vehicle.mode = dronekit.VehicleMode("LAND")
        return True

    def stop(self):
        self.trace("stop")
        return True

    def rtl(self):
        self.trace("rtl")
        self.vehicle.mode = dronekit.VehicleMode("RTL")

    def reset(self):
        self.trace("reset")
        self.shutdown()
        self.initialize()
        return True

    def shutdown(self):
        self.trace("shutdown")
        if self.vehicle is not None:
            self.vehicle.close()
            self.vehicle = None
        if self.mavproxy is not None:
            self.mavproxy.stop()
            if self.debug:
                sys.stderr.write("mavproxy with pid {0} killed\n".format(self.mavproxy.getpid()))
            self.mavproxy = None
        if self.sitl is not None:
            self.sitl.stop()
            if self.debug:
                sys.stderr.write("sitl with pid {0} killed\n".format(self.sitl.getpid()))
            self.sitl = None
        #self.sitl.stop()    # terminates SITL


    def goToW(self, vx, vy, vz, wx, wy, wz, dur):
        self.trace("goToW")
        drone_utils.send_global_velocity(self, (float(vy) + float(wy)), (float(vx) + float(wx)), 0.0, dur)

    def data(self):
        retval = {}
        if self.vehicle is not None and self.vehicle.location.local_frame.north is not None:
            retval['north'] = self.vehicle.location.local_frame.north
            retval['east'] = self.vehicle.location.local_frame.east
            retval['alt'] = -self.vehicle.location.local_frame.down
            retval['auxVel'] = self.vehicle.velocity
            retval['bat'] = self.vehicle.battery.level
            retval['dx'] = retval['auxVel'][0]
            retval['dy'] = retval['auxVel'][1]
            retval['dz'] = retval['auxVel'][2]
            retval['vel'] = math.sqrt(math.pow(retval['dx'], 2) +
                                      math.pow(retval['dy'], 2) +
                                      math.pow(retval['dz'], 2))
            if retval['vel'] != 0:
                retval['dx'] /= retval['vel']
                retval['dy'] /= retval['vel']
                retval['dz'] /= retval['vel']
        return retval

    def __str__(self):
        d = self.data()
        if len(d) > 0:
            return '{0} {1} {2} {3} {4} {5} {6} {7}'.format(d['east'], d['north'], d['alt'], d['dx'], d['dy'], d['dz'], d['vel'], d['bat'])
        else:
            return 'Uninitialized'


    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        self.trace("send_ned_velocity")
        drone_utils.send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration)

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        """
        self.trace("send_global_velocity")
        drone_utils.send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration)

"""
from sitl_drone import *

x = SitlDrone("hello")
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
