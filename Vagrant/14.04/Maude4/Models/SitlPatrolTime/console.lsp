(import "sys")

(define stderr sys.stderr)

(invoke stderr "write" "very\n\tinteresting, \"not\"\n")

