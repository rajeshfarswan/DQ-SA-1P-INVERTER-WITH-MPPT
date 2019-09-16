;first order filter
;all variables are in between +0.5 to -0.5
;modify _FOF_PreOut int variable
;return is int
;input argument1 is input value
;input argument2 is filter constant
;filter constant = (1-k) = e-(2*3.14*Fc*Ts)

#include "p30f6010A.inc"

.global _asmFO_Filter

_asmFO_Filter:
disi #0x3FFF

MOV _FOF_PreOut, W2 ;prevalue in w2
SUB W0,W2,W0 

CALL _asmINT_MPQ
ADD W0,W2,W0

disi #0x0000

return 
.end
