/*

from code.drone import *
x = SimpleDrone("hello")
x.initialize(-7.162675,-34.817705,36,10,10)
x.mv(-7.163147,-34.818550,2,23)

*/



(import "code.drone")

(define SimpleDrone code.drone.SimpleDrone)


(define test ()
  (define xDrone (apply SimpleDrone "xDrone"))

  (invoke xDrone "initialize" (float -7.162675) (float -34.817705)  (int 36) (int 10) (int 10))

  (invoke  xDrone "mv" (float -7.162675) (float -34.817705) (int 2) (int 23))

  (boolean True)
  
  )

;;(invoke xDrone "exit")



(import "plambda.actors.actorlib")

(define send plambda.actors.actorlib.send)

(import "plambda.actors.pyactor")

(define myself (getattr plambda.actors.pyactor.Main "myself"))

(define evalOK (sender val)
  (apply send sender (getattr myself "name") (concat "OK " val "\n"))
  )




