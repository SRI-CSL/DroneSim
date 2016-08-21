from sitl_drone import SimpleDrone

drone0 = SimpleDrone("drone0", 0)

drone0.initialize()

str(drone0.vehicle.battery)

drone0.takeOff(60)

drone0.mv(30, 30, 30, 3)

str(drone0.vehicle.location.global_relative_frame)

str(drone0.vehicle.location.local_frame)

drone0.rtl()

#essentially 

drone0.reset()

#consists of:

drone0.shutdown()

drone0.initialize()

# -7.1626749,-34.8177048
# -7.1624048,-34.8174353


#has no effect 
def recharge(drone):
    drone.vehicle.battery.voltage = 12.587
    drone.vehicle.battery.current = 0.0
    drone.vehicle.battery.level = 100

