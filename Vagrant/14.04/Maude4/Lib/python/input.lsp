(import 'sitl_drone')

(define sitlDrone sitl_drone.SitlDrone)

(define d0
  (let ((largs (mklist "drone_0"))
        (dargs (mkdict "instance_no" (int 0) "debug" (boolean True) "speedup" (int 5))))
    (kwapply sitlDrone largs dargs)))


(invoke d0 'initialize')

(invoke d0 'takeOff' (int 5))

(invoke d0 'mv' (int 200) (int 200) (int 5) (int 3))

(invoke d0 'shutdown')
