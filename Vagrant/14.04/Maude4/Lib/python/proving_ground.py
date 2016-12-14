from dronekit_sitl import SITL

#https://github.com/dronekit/dronekit-python/blob/source-system-filtering/examples/avoidance/avoidance.py#L23


BINARY = '/home/vagrant/Repositories/ardupilot/build/sitl/bin/arducopter-quad'

PARAMS = '/home/vagrant/Repositories/ardupilot/Tools/autotest/default_params/copter.parm'

sitl = SITL(instance=0, path=BINARY, defaults_filepath=PARAMS)

print sitl



"""
sitl.download(system, version, verbose=False) # ...or download system (e.g. "copter") and version (e.g. "3.3")
sitl.launch(args, verbose=False, await_ready=False, restart=False)
sitl.block_until_ready(verbose=False) # explicitly wait until receiving commands
code = sitl.complete(verbose=False) # wait until exit
sitl.poll() # returns None or return code
sitl.stop() # terminates SITL
"""
