(import "drones.simple_drone_sitl-ian")

(define SimpleDrone drones.simple_drone_sitl-ian.SimpleDrone)

(define mkdrone (name x y z v e)
  (let ((drone (apply SimpleDrone name)))
    (invoke drone "initialize" x y z v e)
    drone))

(import "plambda.actors.actorlib")

(define send plambda.actors.actorlib.send)

(import "plambda.actors.pyactor")

(define myself (getattr plambda.actors.pyactor.Main "myself"))

(define evalOK (sender val)
  (apply send sender (getattr myself "name") (concat "OK " val "\n"))
  )
