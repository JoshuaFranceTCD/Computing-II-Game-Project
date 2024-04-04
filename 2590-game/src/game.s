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

  

  .section .text
  
  .equ    FLASH_OFF_TIMER   , 1500
  .equ    FLASH_ON_TIMER   , 500
  
  //.equ    BLINK_PERIOD, FLASH_OFF   @ blink time = 1000 ms
  

Main:
  PUSH  {R4-R5,LR}


  @
  @ Prepare GPIO Port E Pin 9 for output (LED LD3)
  @ We'll blink LED LD3 (the orange LED)
  @
  @ Enable GPIO port E by enabling its clock
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]
  
  


  @ Initialise the first countdown
  LDR     R4, =blink_countdown
  LDR     R5, =FLASH_OFF_TIMER
  STR     R5, [R4]  
  @ Configure SysTick Timer to generate an interrupt every 1ms
  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @
  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
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



.equ     currentPin, LD4_PIN @ set currentPin to LD3_PIN
  BL      enableLED          @ enable the currentPin

Idle_Loop:
  B     Idle_Loop
End_Main:
  POP   {R4-R5,PC}









/*Subroutines & Interrupts */









  @ enableLed subroutine
  @ enables the desired LED for output
  @ parameters: 
  @   R0, LED Label
  @ return

enableLED:
  @  Configure LD3 for output
  @   by setting bits 27:26 of GPIOE_MODER to 01 (GPIO Port E Mode Register)
  @   (by BIClearing then ORRing)
  PUSH    {R4-R5,LR}
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                    @ Read ...
  BIC     R5, #(0b11<<(currentPin*2))    @ Modify ...
  ORR     R5, #(0b01<<(currentPin*2))    @ write 01 to bits 
  STR     R5, [R4]                    @ Write 
  POP    {R4-R5,PC}










@
@ SysTick interrupt handler (blink currentLED)
@
  .type  SysTick_Handler, %function
SysTick_Handler:
  PUSH  {R4, R5, LR}
  LDR   R4, =blink_countdown        @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @
  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @
  B     .LendIfDelay                @ }
.LelseFire:                         @ else {
  LDR     R6, = reacted             @
  LDR     R7,  [R6]
  CMP     R7,#1                     @   )
  BEQ     .LskipInvert              @   if(reacted)
  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @   R5 = boolean on/off
  EOR     R5, #(0b1<<(currentPin))  @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @   R5 = !R5
.LskipInvert:
  CMP     R5, #(0b1<<(currentPin))
  BEQ     .LflashOn
  LDR     R4, =blink_countdown      @   countdown = FLASH_OFF;:
  LDR     R5, = FLASH_OFF_TIMER
  STR     R5, [R4]  
  B       .LendIfDelay
.LflashOn:
  LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;:
  LDR     R5, = FLASH_ON_TIMER
  STR     R5, [R4] 
                  @
.LendIfDelay:                       @ }
  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @
  @ Return from interrupt handler
  POP  {R4, R5, PC}






@
@ External interrupt line 0 interrupt handler
@   (Check if player reacted to LED)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:
  PUSH  {R4,R5,LR}
  LDR   R4, =GPIOE_ODR
  LDR   R5, [R4]
  CMP   R5, #(0b1<<(currentPin))
  BNE   .LledNotOn
  LDR   R4, = reacted
  MOV   R5, #1
  STR   R5, [R4]
  .LledNotOn:
  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @
  @ Return from interrupt handler
  POP  {R4,R5,PC}


  .section .data
  


blink_countdown:
  .space  4
reacted:
  .space 4



  .end