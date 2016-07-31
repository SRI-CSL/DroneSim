/*

from code.vivek_drone import *
x = SimpleDrone("vivek")
x.initialize(-7.162675,-34.817705,36,10,10)

x

x.mv(-7.163147,-34.818550,2,23)
print x.vehicle.location.local_frame
x.mv(7.163147,3.818550,2,230)
print x.vehicle.location.local_frame
time.sleep(1)
x.mv(70.163147,30.818550,2,230)
print x.vehicle.location.local_frame
time.sleep(1)
x.mv(700.163147,300.818550,2,230)
print x.vehicle.location.local_frame


*/




(import "code.vivek_drone")

(import "sys")

(define SimpleDrone code.vivek_drone.SimpleDrone)

(define xDrone (apply SimpleDrone "xDrone"))

(invoke xDrone "initialize" (float -7.162675) (float -34.817705)  (int 36) (int 10) (int 10))

(invoke sys.stderr "write" (apply str xDrone))

(invoke xDrone "mv" (float -7.162675) (float -34.817705)  (int 2) (int 23))



(invoke sys.stderr "write" (apply str xDrone.vehicle.location.local_frame))

(invoke xDrone "mv" (float 7.163147) (float 3.818550)  (int 2) (int 230))

(invoke xDrone "mv" (float 70.163147) (float 30.818550)  (int 2) (int 230))

(invoke xDrone "mv" (float 700.163147) (float 300.818550)  (int 2) (int 230))
