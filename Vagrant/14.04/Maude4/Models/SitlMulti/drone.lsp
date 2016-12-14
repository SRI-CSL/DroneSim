(import "simple_drone")

(define SimpleDrone simple_drone.SimpleDrone)

;; at some stage the x y z v e will be used.
;; currently we drop them on the floor.
;;
(define mkdrone (name ins)
  ;; the 0 is the instance number (all ports depend on it)
  (let ((drone (apply SimpleDrone name ins)))
    (setuid drone name)
    (invoke drone "initialize")
    drone))

(define logging (boolean true))
(define logFile "log10.txt")

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

(define setStartTime () (define startTime (invoke time "clock")))

(define getTimeElapsed ()
  (* (- (invoke time "clock") startTime ) (int 1000)))


(import "plambda.util.Util")

(define writeLog (name)
  (apply plambda.util.Util.string2File (concat (invoke name "getName") " " (apply getTimeElapsed) " " name " \n\n") logFile (boolean true)))
