# PASTE LINK TO TEAM VIDEO BELOW
#
#

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 250

  .section .text

Main:
  LDR  R0, = 0x20000018
  LDR  R6, = 0x20000019 
  LDR  R10, =0x20000010
  LDR  R11, =0x20000014

  PUSH  {R4-R12,LR}
  MOV   R7, #8
  MOV   R8, #0           @ boolean correctLight = false;
  MOV   R3, #100

  @
  @ Prepare GPIO Port E Pin 9 for output (LED LD3)
  @ We'll blink LED LD3 (the orange LED)
  @

  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]

  @ Configure LD3 for output
  @   by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @   (by BIClearing then ORRing)
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD4_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD4_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD5_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD5_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD6_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD6_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD7_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD7_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD8_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD8_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD9_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD9_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD10_PIN*2))    @ Modify ...
  ORR     R5, #(0b01<<(LD10_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 

  @ Initialise the first countdown

  LDR     R4, =blink_countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5,= #7999                    @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1


  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt Line0
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on Line0
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt #6 (external interrupt Line0)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]
  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  LDR   R9, [R11]
  CMP   R9, #1
  BEQ   .Lfail
  B     Idle_Loop

.Lfail:
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD3_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD4_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD4_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD5_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD5_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD6_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD6_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD7_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD7_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD8_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD8_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD9_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD9_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(LD10_PIN*2))    @ Modify ...
  ORR     R5, #(0b00<<(LD10_PIN*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
End_Main:
  POP   {R4-R12,PC}



@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4-R12, LR}

  LDR   R4, =blink_countdown        @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @
  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {
  LDR      R0, =0x20000018
  LDRB     R1, [R0]
  CMP     R1, #0 
  BNE     .LLDFive
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 
  ADD     R1, R1, #1
  STRB     R1, [R0]
  LDRB     R2, [R6]
  CMP      R2, #0
  BEQ     .LdoneFlash
  EOR     R5, #(0b1<<(LD4_PIN)) 
  STR     R5, [R4]
  B       .LdoneFlash
 .LLDFive:
  LDR    R0, =0x20000018
  LDRB     R1, [R0]
  CMP     R1, #1
  BNE     .LLDSeven
   LDR     R4, =GPIOE_ODR            @   Invert LD3
   LDR     R5, [R4]                  @
   EOR     R5, #(0b1<<(LD5_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
   EOR     R5, #(0b1<<(LD3_PIN))   
   STR     R5, [R4]                  @ 
   ADD     R1, R1, #1
   STRB     R1, [R0]
   B       .LdoneFlash
  .LLDSeven:
  LDR    R0, =0x20000018
  LDRB    R1, [R0]
  CMP     R1, #2
  BNE     .LLDNine
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD7_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  EOR     R5, #(0b1<<(LD5_PIN))   
  STR     R5, [R4]                  @ 
  ADD     R1, R1, #1
  STRB     R1, [R0]
  B       .LdoneFlash
  .LLDNine:
  LDR  R0, =0x20000018
  LDRB     R1, [R0]
  CMP     R1, #3
  BNE     .LLDTen
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD9_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  EOR     R5, #(0b1<<(LD7_PIN))   
  STR     R5, [R4]                  @ 
  ADD     R1, R1, #1
  STRB    R1, [R0]
  B       .LdoneFlash
  .LLDTen:
  LDR    R0, =0x20000018
  LDRB     R1, [R0]
  CMP     R1, #4
  BNE     .LLDEight
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD10_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  EOR     R5, #(0b1<<(LD9_PIN))
  STR     R5, [R4]                  @ 
  MOV     R8, #1                    @ correctLight = true;
  STR     R8, [R10]
  ADD     R1, R1,  #1 
  STRB     R1, [R0]
  B       .LdoneFlash
  .LLDEight:
  LDR  R0, =0x20000018
  LDRB     R1, [R0]
  CMP     R1, #5
  BNE     .LLDSix
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD8_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  EOR     R5, #(0b1<<(LD10_PIN))
  STR     R5, [R4]                  @ 
  MOV     R8, #0                    @ correctLight = false;
  STR     R8, [R10]
  ADD     R1, R1, #1
  STRB    R1, [R0]
  B       .LdoneFlash
  .LLDSix:
  LDR    R0, =0x20000018
  LDRB     R1, [R0]
  CMP     R1, #6
  BNE     .LLDFour
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD6_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  EOR     R5, #(0b1<<(LD8_PIN)) 
  STR     R5, [R4]                  @ 
  ADD     R1, R1, #1
  STRB     R1, [R0]
  B       .LdoneFlash
  .LLDFour:
  LDR  R0, =0x20000018
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD4_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  EOR     R5, #(0b1<<(LD6_PIN)) 
  STR     R5, [R4]                  @ 
  MOV     R1,  #0
  STRB     R1, [R0]
.LdoneFlash:
  LDRB    R2, [R6]
  ADD     R2, #1
  STRB    R2, [R6]  
  @ MOV     R4, #160
  @ CMP     R7, R4
  @ BHI     .LStopIncrease
  @ CMP     R2, R7
  @ BLO     .LStopIncrease
  @ SUB     R3, R3, #4000
  @ LDR     R4, =SYSTICK_LOAD         @ Set SysTick LOAD for 1ms delay
  @ MOV     R5, R3                    @ Assuming 8MHz clock
  @ STR     R5, [R4]     
  @ ADD     R7, R7, #8
  @ .LStopIncrease:
  LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD         @
  SUB     R5, R5, R3
  STR     R5, [R4]  
  ADD     R3, R3, #50
  MOV     R5, #8
  UDIV    R12, R2, R5
  LDR     R4, =button_count             @ count = 0;
  LDR     R5, [R4]
  CMP     R5, R12
  BLO     .Lfail
  
.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @
  @ B       SysTick_Handler
  @ Return from interrupt handler
  POP  {R4-R12, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4-R12, LR}

  LDR   R4, =button_count
  LDR   R5, [R4]
  ADD   R5, R5, #1
  STR   R5, [R4]
  LDR   R10, =0x20000010            @ 
  LDR   R11, =0x20000014            @
  LDR   R8, [R10]                   @
  CMP   R8, #1                      @
  BEQ   .LcontinueProgram           @ 
  MOV   R9, #1                      @ boolean failedTiming = true;
  STR   R9, [R11]                   @

.LcontinueProgram:
  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @
  @ Return from interrupt handler
  POP  {R4-R12, PC}
  
  .section .data

button_count:
  .space  4

blink_countdown:
  .space  4

memory_for_game:
  .space 24

  .end