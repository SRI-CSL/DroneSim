
;;;Editing is Futile: autogenerated by "controller.lsp"
(load "/home/vagrant/Repositories/imaude/jlib/jl-util.lsp")
(load "../../../../Lib/stats-funs.lsp")
(define generic_alpha (double 0.25))
(define generic_beta (double 0.25))
(define generic_num (int 5))
(define ssobj (object ("g2d.stats.Synchronize" (object ("g2d.stats.nGenericApprox" "Patrol" generic_alpha generic_beta generic_num)))))
(define maudes (array java.lang.String "maude0" "maude1" "maude2" ))
(invoke java.lang.System.err "println" "The g2d actor is (auto) initialized. Executing doConcurrentStats Now.
")
(apply doConcurrentStats "Patrol" maudes)
