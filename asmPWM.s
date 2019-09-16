/* Full-Bridge PWM and MPPT PWM generating routine */

;use variable "currentP_Dout" for inverter duty cycle 1 and 2
;use variable "final_ref" for MPPT converter duty cycle 3
;alter processor dependent duty cycle registers
;use variable PWM_max as offset


#include "p30f6010A.inc"

.global _asmPWM

_asmPWM:
disi #0x3FFF

MOV _PWM_offset, W5 ;PWM ofsset value

MOV _currentP_Dout, W0 ; copy PI output to duty D
SUB W5,W0,W6 ;1-D in w6;
ADD W0,W5,W0
CALL _asmLT0 ;check less than zero
MOV W0, PDC1 ;program duty1

MOV W6,W0
ADD W0,W5,W0
CALL _asmLT0 ;check less than zero
MOV W0, PDC2 ;program duty2

MOV _final_ref, W0 ; copy MPPT voltage ref
ADD W0,W5,W0
CALL _asmLT0 ;check less than zero
MOV W0, PDC3 ;program duty3

BRA Last2

;check value for less than zero
_asmLT0:
CP0 W0
BRA GT,Last1
MOV #0x0000,W0

Last1:
return
;less than zero function ends

Last2:
disi #0x0000
return
.end



