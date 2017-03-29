(define plambda_init_lsp
  (concat
   "\n"
   ";;;Editing is Futile: autogenerated by 'controller-scp.lsp'"
   "\n"
   "(import 'sys')"
   "\n"
   "(invoke sys.stderr 'write' 'initializing {0}\\n')"
   "\n"
   "(load '../../../../Lib/sitl_drone.lsp')"
   "\n"
   "(define b0 (apply mkdrone 'b0' (* (int 4) (int {1})) (int 0) (int 0) (boolean false) (boolean false) 'log0.txt' ))"
   "\n"
   "(define b1 (apply mkdrone 'b1' (+ (* (int 4) (int {1})) (int 1)) (int 100) (int 100) (boolean false) (boolean false) 'log1.txt' ))"
   "\n"
   "(define b2 (apply mkdrone 'b2' (+ (* (int 4) (int {1})) (int 2)) (int 10) (int 10) (boolean false) (boolean false) 'log1.txt' ))"
   "\n"
   "(define b3 (apply mkdrone 'b3' (+ (* (int 4) (int {1})) (int 3)) (int 110) (int 110) (boolean false) (boolean false) 'log1.txt' ))"
   "\n"
   "(apply setStartTime)"
   "\n"
   "(import 'plambda.actors.actorlib')"
   "\n"
   "(apply plambda.actors.actorlib.send 'plambda' 'plambda{1}' 'initedOK plambda{1}')"
   "\n"
   )
  )

(define maude_init_maude
  (concat
   "\n"
   "**** Editing is Futile: autogenerated by 'controller-scp.lsp'"
   "\n"
   "load load-scp"
   "\n"
   "load model-checker"
   "\n"
   "load /home/vagrant/Repositories/imaude/ilib/load-lib"
   "\n"
   "load ../../../../Lib/meta-cp-e"
   "\n"
   "loop init ."
   "\n"
   "(seq\n (initAgentEset maude{0} g2d g2d plambda{0} ANALYSIS asysNoWind40 false)\n (augAgentEsetConcurrentStats Patrol 400))"
   "\n"
   )
  )

(define clone_count (int 2))

;;ian thinks this is not used:
(define num_drones (int 4))



(define generic_alpha "0.5")
(define generic_beta "0.5")
(define generic_num  "5")

;;; NO MORE EDITING FROM THIS POINT


(define g2d_init_lsp
  (concat
   '\n'
   ';;;Editing is Futile: autogenerated by "controller.lsp"'
   '\n'
   '(load "/home/vagrant/Repositories/imaude/jlib/jl-util.lsp")'
   '\n'
   '(load "../../../../Lib/stats-funs.lsp")'
   '\n'
   '(define generic_alpha (double {1}))'
   '\n'
   '(define generic_beta (double {2}))'
   '\n'
   '(define generic_num (int {3}))'
   '\n'
   '(define ssobj (object ("g2d.stats.Synchronize" (object ("g2d.stats.nGenericApprox" "Patrol" generic_alpha generic_beta generic_num)))))'
   '\n'
   '(define maudes (array java.lang.String {0}))'
   '\n'
   '(invoke java.lang.System.err "println" "The g2d actor is (auto) initialized. Executing doConcurrentStats Now.\n")'
   '\n'
   '(apply doConcurrentStats "Patrol" maudes)'
   '\n'
   )
  )

(define debug (boolean False))

(import 'sys')
(define logmsg (msg)
  (if debug
      (invoke  sys.stderr 'write' msg)
    )
  )

(define plambda_clones (mklist))
(define plambda_population clone_count)

(define maude_clones (mklist))
(define maude_population clone_count)



(import 'plambda.util.StringBuffer')
;;;makes a string of the names of count maudes: ' "maude0" "maude1" "maude2" ... "maude<count - 1>" '
(define make_maudes_string (count)
  (let ((sb (apply plambda.util.StringBuffer.StringBuffer)))
    (for index count
	 (invoke sb 'append' '"maude')
	 (invoke sb 'append' index)
	 (invoke sb 'append' '" ')
	 )
    (apply str sb)
    )
  )

(import 'plambda.util.Util')
;; instantiates the template g2d_init_lsp and writes it out to g2dinit.lsp
(define make_g2dinit_lsp ()
  (let  ((lspfile 'g2dinit.lsp')
	 (contents (invoke g2d_init_lsp
			   'format'
			   (apply make_maudes_string maude_population)
			   generic_alpha
			   generic_beta
			   generic_num)))
    (apply plambda.util.Util.string2File  contents lspfile (boolean False))
    )
  )

;;make/write the g2dinit.lsp file
(apply make_g2dinit_lsp)

;;makes the argument string for the startup command for the actor called prefix<instanceno>
(define make_args (prefix instanceno)
  (if (= prefix 'plambda')
      (concat  ' plambda' instanceno)
    (concat ' basura load'instanceno)))

(import 'sys')
(import 'plambda.actors.actorlib')
;;
;; make_cloner is used to create a handler that will respond by cloning
;; a collection of actor clones. for example:
;;
;;       (apply make_cloner 'plambda' 'pyactor' plambda_clones plambda_population))
;;       (apply make_cloner 'maude' 'iop_maude_wrapper' maude_clones maude_population))
;;
(define make_cloner (prefix executable clones count)
  (lambda (sender message)
    (apply sys.stderr.write (concat 'cloner for ' prefix ' got ' message ' from ' sender '\n'))
    (let ((tokens (invoke message 'split')))
      (if (> (apply len tokens) (int 1))
	  (let ((verb (get tokens (int 0)))
		(noun (get tokens (int 1))))
	    (if (and (= verb 'startOK') (invoke noun 'startswith' prefix))
		(seq
		 (apply sys.stderr.write 'it is for us\n')
		 (invoke clones 'append' noun)
		 (if (< (apply len clones) count)
		     (seq
		      (apply logmsg (concat 'Handled ' noun ' more coming\n'))
		      (let ((instanceno (apply len clones))
			    (name (concat prefix instanceno))
			    (args (apply make_args prefix instanceno)))
			(apply plambda.actors.actorlib.send
			       'system'
			       'plambda'
			       (concat 'start ' name ' ' executable ' ' args)))
		      )
		   (seq (apply logmsg (concat 'Handled ' noun ' enough already\n'))
			(apply init_clones clones prefix)
			)
		   )
		 (boolean true)
		 )
	      (seq
	       (boolean false)
	       )
	      )
	    )
	)
      )
    )
  )

(define inited_clones (mklist))

(define init_handler
  (lambda (sender message)
    (let ((tokens (invoke message 'split')))
      (if (> (apply len tokens) (int 1))
	  (let ((verb (get tokens (int 0)))
		(noun (get tokens (int 1))))
	    (if (and (= verb 'initedOK') (invoke noun 'startswith' 'plambda'))
		(seq
		 (invoke inited_clones 'append' noun)
		 (if (= (apply len inited_clones) plambda_population)
		     (seq
		      (apply plambda.actors.actorlib.send 'g2d' 'plambda' '(load "g2dinit.lsp")')
		      (apply logmsg 'Sent initialize message to g2d\n')
		      )
		   )
		 (boolean True)
		 )
	      )
	    )
	(boolean False)
	)
      )
    )
  )

(import 'time')
(define init_clones (clones prefix)
  (if (= prefix 'plambda')
      (seq
       (for clone clones
	    (let ((loadfile (concat 'init_' clone '.lsp'))
		  (contents (invoke plambda_init_lsp 'format' clone (invoke clones 'index' clone))))
	      (apply plambda.util.Util.string2File  contents loadfile (boolean False))
	      (apply plambda.actors.actorlib.send clone 'plambda' (concat '(load "' loadfile '")'))
	      )
	    )
       (apply plambda.actors.actorlib.send
	      'system'
	      'plambda'
	      (concat 'start maude0 iop_maude_wrapper ' (apply make_args 'maude' (int 0))))
       )
    )
  (if (= prefix 'maude')
      (seq
       (for clone clones
	    (seq
	     (apply logmsg (concat 'clone: ' clone ' with prefix ' prefix '\n'))
	     )
	    )
       )
    )
  )



(define catchall
  (lambda (sender message)
    (invoke  sys.stderr 'write' (concat 'Handled: ' message ' from ' sender '\n'))))

(import 'plambda.actors.pyactor')

(apply plambda.actors.pyactor.add_handler
       (apply make_cloner 'plambda' 'pyactor' plambda_clones plambda_population))

(apply plambda.actors.pyactor.add_handler
       (apply make_cloner 'maude' 'iop_maude_wrapper' maude_clones maude_population))

(apply plambda.actors.pyactor.add_handler init_handler)

;;make the maude load files.
(for instanceno maude_population
     (let  ((loadfile (concat 'load' instanceno '.maude'))
	    (contents (invoke maude_init_maude 'format' instanceno)))
       (apply plambda.util.Util.string2File  contents loadfile (boolean False))))




;; start the ball rolling
;;
;; asks the system to create the first pyactor called plambda0
;;
;; which when successful will respond to plambda with 'initedOK plambda0'
;; this will trigger the 'init_handler'
;;
(let ((name (concat 'plambda' (int 0))))
  (apply plambda.actors.actorlib.send 'system' 'plambda' (concat 'start ' name '  pyactor ' name))
  )

(invoke  sys.stderr 'write' 'Ball now rolling\n')

/*

This script, run by the 'controller' actor, an pyactor instance that starts up from
the init script (startup-sc-concurrent.txt), along with the g2d actor, and of course the system actor.
It is parametric on the important number:

(define clone_count (int N + 1))

and is designed to create a system of actors that includes itself, g2d,  AND:

plambda0  .... plambdaN

maude0 .... maudeN





It configures itself first by adding several handlers, then
starts the ball rolling by asking the system to create the first plambda clone.



*/
