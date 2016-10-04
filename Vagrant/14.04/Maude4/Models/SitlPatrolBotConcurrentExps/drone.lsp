(import "sitl_drone")

(define SimpleDrone sitl_drone.SimpleDrone)

;; at some stage the x y z v e will be used.
;; currently we drop them on the floor.
;;
(define mkdrone (name ino)
  ;; the 0 is the instance number (all ports depend on it)
  (let ((drone (apply SimpleDrone name ino)))
    (setuid drone name)
    (invoke drone "initialize")
    drone))

(import "plambda.actors.actorlib")

(define send plambda.actors.actorlib.send)

(import "plambda.actors.pyactor")

(define myself (getattr plambda.actors.pyactor.Main "myself"))

(define evalOK (sender val)
  (apply send sender (getattr myself "name") (concat "OK " val "\n"))
  )

(define console ()
  (setattr plambda.actors.pyactor.Main "launchConsole" (boolean True))
  )

(define resetSitl (idlist)
  (for id idlist (apply resetSiltAux  id))
  (boolean True)
  )

(import "sys")

(define resetSiltAux (id)
  (let ((sitlobj (fetch id)))
    (if (isobject sitlobj)
	(invoke sitlobj "reset")
      (invoke sys.stderr "write"  (concat "no object found for id: " id))
      )
    )
  )

(import "time")


