from sitl_drone import SimpleDrone

drone0 = SimpleDrone("drone0", 0)

drone0.initialize()

drone0.takeOff(60)

drone0.mv(30, 30, 30, 3)

str(drone0.vehicle.location.global_relative_frame)

drone0.rtl()

#does not reset the battery level.
drone0.reset()

drone0.shutdown_and_exit()

# -7.1626749,-34.8177048
# -7.1624048,-34.8174353
