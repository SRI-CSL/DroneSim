(load "/home/vagrant/imaude/jlib/jl-util.lsp")
(load "stats-funs.lsp")

(define ssobj (object ("g2d.stats.Synchronize" (object ("g2d.stats.nGenericApprox" "Patrol" (double 0.5) (double 0.5) (int 4))))))
;;(define ssobj (object ("g2d.stats.nGenericApprox" "Patrol" (double 0.5) (double 0.5) (int 4))))

(define maudes (array java.lang.String "maude0" "maude1"))

(apply doConcurrentStats "Patrol" maudes)

(invoke java.lang.System.err "println" "The g2d actor is initialized!")


