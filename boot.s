.cpu cortex-m7
.fpu softvfp
.syntax unified
.thumb

.section .vectors,"a",%progbits     // Cortex-M7 interrupt handlers

.word   _estack              // 0 Stack top address
.word   _reset               // 1 Reset
.word   halt                 // 2 NMI
.word   halt                 // 3 Hard Fault
.word   halt                 // 4 MM Fault
.word   halt                 // 5 Bus Fault
.word   halt                 // 6 Usage Fault
.word   halt                 // 7 RESERVED
.word   halt                 // 8 RESERVED
.word   halt                 // 9 RESERVED
.word   halt                 // 10 RESERVED
.word   halt                 // 11 SV call
.word   halt                 // 12 Debug reserved
.word   halt                 // 13 RESERVED
.word   halt                 // 14 PendSV
.word   halt                 // 15 SysTick
                             // 98 STM32 handlers
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,ETH_IRQHandler,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt,halt,halt
.word   halt,halt,halt,halt,halt,halt,halt,halt

.section .text
.global _reset
.thumb_func
_reset:
  ldr sp, =_estack
  bl main
  b .

.thumb_func
halt:   b .

.thumb_func
pass:   bx lr
