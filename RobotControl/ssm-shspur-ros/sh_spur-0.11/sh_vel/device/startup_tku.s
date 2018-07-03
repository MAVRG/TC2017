    .section    .text
    .global     _start

_start:
    MOV.L    0x0,    r15   !stack base

    MOV.L       _main, r0  !jump
    JSR         @r0      
    OR          r0,     r0

LOOP:
    BRA         LOOP
    OR          r0,     r0

.end
	