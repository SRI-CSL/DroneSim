(import "sitl_drone")

(define SimpleDrone sitl_drone.SimpleDrone)

;; at some stage the x y z v e will be used.
;; currently we drop them on the floor.
;;
(define mkdrone (name x y z v e)
  ;; the 0 is the instance number (all ports depend on it)
  (let ((drone (apply SimpleDrone name (int 0))))
    (invoke drone "initialize")
    drone))

(import "plambda.actors.actorlib")

(define send plambda.actors.actorlib.send)

(import "plambda.actors.pyactor")

(define myself (getattr plambda.actors.pyactor.Main "myself"))

(define evalOK (sender val)
  (apply send sender (getattr myself "name") (concat "OK " val "\n"))
  )
