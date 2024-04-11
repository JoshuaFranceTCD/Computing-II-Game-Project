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
  
  .equ    FLASH_OFF_TIMER  , 1500      // length of time the flash remains of(static)
  .equ    FLASH_ON_TIMER   , 500       // length of time the light flashes on (changes depending on the level)
Main:
  PUSH  {R4-R5,LR}


  /*************************************   SET UP  *************************************************8*/
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



/****************************************** Main Program *************************************************/

  MOV    R6,#0                @ create points variable

.equ     currentPin, LD3_PIN  @ set currentPin to LD3_PIN
  BL      enableLED           @ enable the currentPin

// loop continously until player reacts correctly
level1:
  LDR   R4, =reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel1
  B     level1
endLevel1:
  ADD   R6,R6,#1                @ level 1 completed, points++;

  .equ     FLASH_ON_TIMER, 200  @ FLASH_ON_TIMER = 200 (flash on timer becomes smaller each time)
  .equ     currentPin, LD4_PIN  @ set currentPin to LD4_PIN
   BL      enableLED            @ enable the currentPin
   MOV   R5,#0
   LDR   R5, [R4]               @ reacted = 0  

level2:
  LDR   R4, = reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel2
  B     level2
endLevel2:
  ADD   R6,R6,#1                @ level 2 completed, points++;

// level 3, 4 etc

  .equ    FLASH_ON_TIMER, 150  @ FLASH_ON_TIMER = 150 (flash on timer becomes smaller each time)
  .equ    currentPin, LD6_PIN  @ set currentPin to LD5_PIN
  BL      enableLED            @ enable the currentPin
  MOV     R5,#0
  LDR     R5, [R4]               @ reacted = 0  

level3:
  LDR   R4, = reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel3
  B     level3
endLevel3:
  ADD   R6,R6,#1                @ level 3 completed, points++;

    .equ    FLASH_ON_TIMER, 100  @ FLASH_ON_TIMER = 100 (flash on timer becomes smaller each time)
  .equ    currentPin, LD8_PIN  @ set currentPin to LD6_PIN
  BL      enableLED            @ enable the currentPin
  MOV     R5,#0
  LDR     R5, [R4]               @ reacted = 0  

level4:
  LDR   R4, = reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel4
  B     level4
endLevel4:
  ADD   R6,R6,#1                @ level 3 completed, points++;

  .equ    FLASH_ON_TIMER, 75  @ FLASH_ON_TIMER =75 (flash on timer becomes smaller each time)
  .equ    currentPin, LD10_PIN  @ set currentPin to LD10_PIN
  BL      enableLED            @ enable the currentPin
  MOV     R5,#0
  LDR     R5, [R4]               @ reacted = 0  

level5:
  LDR   R4, = reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel5
  B     level5
endLevel5:
  ADD   R6,R6,#1                @ level 3 completed, points++;

  .equ    FLASH_ON_TIMER, 50 @ FLASH_ON_TIMER = 50 (flash on timer becomes smaller each time)
  .equ    currentPin, LD9_PIN  @ set currentPin to LD9_PIN
  BL      enableLED            @ enable the currentPin
  MOV     R5,#0
  LDR     R5, [R4]               @ reacted = 0  

level6:
  LDR   R4, = reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel6
  B     level6
endLevel6:
  ADD   R6,R6,#1                @ level 3 completed, points++;

  .equ    FLASH_ON_TIMER, 35  @ FLASH_ON_TIMER = 35 (flash on timer becomes smaller each time)
  .equ    currentPin, LD7_PIN  @ set currentPin to LD7_PIN
  BL      enableLED            @ enable the currentPin
  MOV     R5, #0
  LDR     R5, [R4]               @ reacted = 0  


level7:
  LDR   R4, = reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel7
  B     level7
endLevel7:
  ADD   R6,R6,#1                @ level 3 completed, points++;

  .equ    FLASH_ON_TIMER, 25  @ FLASH_ON_TIMER = 25 (flash on timer becomes smaller each time)
  .equ    currentPin, LD5_PIN  @ set currentPin to LD5_PIN
  BL      enableLED            @ enable the currentPin
  MOV     R5,#0
  LDR     R5, [R4]               @ reacted = 0  


level8:
  LDR   R4, = reacted 
  LDR   R5, [R4]
  CMP   R5, #1
  BEQ   endLevel8
  B     level8
endLevel8:
  ADD   R6,R6,#1                @ level 3 completed, points++;






// C. Quinn, created ending sequence, 13:00, 06/04/2024
  CMP       R6,#1                   @ if score > 1, flash first LED
  BLS       End_Main
  .equ      currentPin, LD3_PIN     @ set currentPin to LD3_PIN
  BL        enableLED               @ enable the currentPin
  CMP       R6,#2                   @ if score > 2, flash second LED
  BLS       End_Main
  .equ      currentPin, LD4_PIN     @ set currentPin to LD3_PIN
  BL        enableLED               @ enable the currentPin
  CMP       R6,#3                   @ if score > 3, flash third LED
  BLS       End_Main
  .equ      currentPin, LD5_PIN     @ set currentPin to LD3_PIN
  BL        enableLED               @ enable the currentPin
  CMP       R6,#4                   @ if score > 4, flash third LED
  BLS       End_Main
  .equ      currentPin, LD6_PIN     @ set currentPin to LD3_PIN
  BL        enableLED               @ enable the currentPin


End_Main:
  POP   {R4-R5,PC}


/*******************************Subroutines & Interrupts **********************/









  @ enableLed subroutine:

  @ enables the desired LED for output
  @ parameters: 
  @   currentPin


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
  BEQ     .LskipInvert              @   if(reacted) Invert LD3L
  LDR     R4, =GPIOE_ODR            @   {
  LDR     R5, [R4]                  @   ledStatus = on/off
  EOR     R5, #(0b1<<(currentPin))  @   GPIOE_ODR = GPIOE_ODR ^ (1<<currentPin);
  STR     R5, [R4]                  @   ledStatus = !ledStatus
.LskipInvert:                       @   }
  CMP     R5, #(0b1<<(currentPin))  @   if(ledStatus = off)
  BEQ     .LflashOn                 @   {
  LDR     R4, =blink_countdown      @     
  LDR     R5, = FLASH_OFF_TIMER     @
  STR     R5, [R4]                  @     countdown = FLASH_OFF_TIMER;
  B       .LendIfDelay              @   }
.LflashOn:
  LDR     R4, =blink_countdown      @   else{
  LDR     R5, = FLASH_ON_TIMER
  STR     R5, [R4]                  @   countdown = FLASH_ON_TIMER;
                                    @
.LendIfDelay:                       @     }
  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @
  @ Return from interrupt handler
  POP  {R4, R5, PC}


@
@ External interrupt line 0 interrupt handler(Push Button Handler)
@   (Check if player pressed the button on time)
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



// memoryAddresses
  .section .data
blink_countdown:
  .space  4
reacted:
  .space 4



  .end