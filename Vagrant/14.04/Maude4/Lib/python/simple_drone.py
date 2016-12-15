import time
import math
import sys
import sandbox
import drone_utils
import dronekit



"""
At some stage we want to pass in the position, altitude and yaw to
the constructor. But currently we are hardcode with

7.162675, -34.817705, 36,      250
latitude  longitude   altitude yaw



"""


tracing = True



class SimpleDrone(object):
    """
    plambda lesson:

    after

    (import "simple_drone")

    can make a drone like so:

    (apply simple_drone.SimpleDrone  "drone_0")

    and get the defaults for instance_no, debug.

    or:

    (let ((largs (mklist "drone_0"))
          (dargs (mkdict "instance_no" (int 666) "debug" (boolean False))))
       (kwapply simple_drone.SimpleDrone largs dargs))

    and get a customized drone whose battery will run out quicker.
    kw stands for keyword.

    """

    def __init__(self, name, instance_no=0, debug=False, speedup=None):
        """Creates a drone with given name and default state.
        """
        self.name = name
        self.debug = debug
        self.ino = instance_no
        self.pipeincr = 10 * self.ino

        self.speedup = int(speedup) if speedup is not None else None

        if self.debug:
            msg = 'SimpleDrone with name {0} and instance number {1}\n'
            sys.stderr.write(msg.format(self.name, self.ino))

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
        self.trace("getName")
        return self.name

    def initialize(self):

        self.spawn()

        self.vehicle = dronekit.connect(self.sitl_ip(), wait_ready=True)

        self.setSpeedup(self.speedup)

        return True

    def sitl_ip(self):
        return '127.0.0.1:{0}'.format(14550 + self.pipeincr)

    def drokekit_args(self):
        dkargs = ['dronekit-sitl',
                  'copter-3.3',
                  '--instance',
                  '{0}'.format(self.ino),
                  '--home=-7.162675,-34.817705,36,250']
        if self.speedup is not None:
            dkargs.append('--speedup={0}'.format(self.speedup))
        return dkargs

    def mavproxy_args(self):
        mpargs = ['mavproxy.py',
                  '--master',
                  'tcp:127.0.0.1:{0}'.format(5760 + self.pipeincr),
                  '--sitl=127.0.0.1:{0}'.format(5501 + self.pipeincr),
                  '--out=127.0.0.1:{0}'.format(14550 + self.pipeincr),
                  '--aircraft',
                  '/tmp/drone_{0}'.format(self.ino)]
        if self.debug:
            mpargs.extend(['--map', '--console'])
        return mpargs


    def trace(self, name):
        if tracing:
            d = self.data()
            if len(d) > 0:
                sys.stderr.write("{0}.{1} mode = {2} vel = {3} alt = {4}\n".format(self.ino, name, self.vehicle.mode, d['vel'], d['alt']))
            else:
                sys.stderr.write("{0}.{1} mode = {2} vel = {3} alt = {4}\n".format(self.ino, name, self.vehicle.mode, 'U', 'U'))


    def spawn(self):
        dkargs = self.drokekit_args()
        if self.debug:
            sys.stderr.write("Spawning dronekit {0}\n".format(dkargs))
        self.dronekit = sandbox.SandBox('dronekit', dkargs, False)
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
        self.mavproxy = sandbox.SandBox('mavproxy', mpargs, True)
        self.mavproxy.start()
        if self.debug:
            sys.stderr.write("mavproxy with pid {0} spawned\n".format(self.mavproxy.getpid()))


    def setSpeedup(self, speedup):
        """Set the speedup of the simulation.

        -- speedup the factor by which we speed up.

        Note Bene: It will NOT work while the drone_kit binaries are out of date.
        """

        try:
            self.speedup = int(speedup) if speedup is not None else None
            if self.speedup is not None:
                while True:
                    self.vehicle.parameters['SIM_SPEEDUP'] = self.speedup
                    sys.stderr.write('Setting SIM_SPEEDUP to {0}\n'.format(self.speedup))
                    if self.vehicle.parameters['SIM_SPEEDUP'] == self.speedup:
                        break
                    time.sleep(0.1)
        except Exception as e:
            sys.stderr.write('simple_drone.SimpleDrone.setSpeedup FAILED (as EXPECTED), USE sitl_drone.SitlDrone\n')


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
        #self.send_global_velocity(0,0,0,1)
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
        if self.dronekit is not None:
            self.dronekit.stop()
            if self.debug:
                sys.stderr.write("dronekit with pid {0} killed\n".format(self.dronekit.getpid()))
            self.dronekit = None

    def goToW(self,vx,vy,vz,wx,wy,wz,dur):
        self.trace("goToW")
        self.send_global_velocity( (float(vy) + float(wy)),(float(vx) + float(wx)),0.0,dur)

    def data(self):
        retval = {}
        if self.vehicle is not None and self.vehicle.location.local_frame.north is not None:
            retval['north'] = self.vehicle.location.local_frame.north
            retval['east'] = self.vehicle.location.local_frame.east
            retval['alt'] =  -self.vehicle.location.local_frame.down
            retval['auxVel'] = self.vehicle.velocity
            retval['bat'] = self.vehicle.battery.level
            retval['dx'] = retval['auxVel'][0]
            retval['dy'] = retval['auxVel'][1]
            retval['dz'] = retval['auxVel'][2]
            retval['vel'] = math.sqrt(math.pow(retval['dx'],2) +
                                      math.pow(retval['dy'],2) +
                                      math.pow(retval['dz'],2))
            if retval['vel'] != 0:
                retval['dx'] /= retval['vel']
                retval['dy'] /= retval['vel']
                retval['dz'] /= retval['vel']
        return retval

    #ian says: why do we have log and __str__?? log is superfluous: log(self) == str(self)
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
x.vehicle.location.local_frame.north
from simple_drone import *

x = SimpleDrone("hello")
x.initialize()

x.takeOff(5)

#"Start Position: "
x.vehicle.location.local_frame

#"Go to Destination 1"
x.mv(200,200,5,3)

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
