(import "sitl_drone")

(define Drone sitl_drone.SitlDrone)

;;
;;  NOTE THE NEW ARGUMENTS TO mkdrone...
;;  I (iam) have only updated Drones/two-drones
;;  anything else will be broke.
;;
;;
;;  delta_x: distance in meters (east) from default home
;;  delat_y: distance in meters (north) from default home
;;
;;
(define mkdrone (name ins delta_x delta_y mapP logP logFileName)
  ;; the 0 is the instance number (all ports depend on it)
  (let ((drone (apply Drone name ins mapP)))
    (setuid drone name)
    ;;
    ;; you can now pass in a delta to the initialize routine.
    ;; x and y should be floats, or strings that look like floats.
    ;; ints or strings that look like ints are ok too.
    ;;
    ;;(invoke drone "initialize" x y)
    ;;
    ;;
	;; the default is x = 0 and y = 0
    (invoke drone "initialize" delta_x delta_y)
    (define logging logP)
    (define logFile logFileName)
    drone))

(import "plambda.actors.actorlib")

(define send plambda.actors.actorlib.send)

;;(setattr plambda.actors.actorlib "debug" (boolean true))


(import "plambda.actors.pyactor")

;;(setattr plambda.actors.pyactor "debug" (boolean true))


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
  (- (invoke time "clock") startTime ) )


(import "plambda.util.Util")

(define writeLog (name)
  (apply plambda.util.Util.string2File (concat (invoke name "getName") " " (apply getTimeElapsed) " " name " \n\n") logFile (boolean true)))
