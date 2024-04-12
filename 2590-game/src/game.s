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
  .equ    FLASH_OFF_TIMER  , 1500      // length of time the flash remains off
  Main:
  PUSH  {R4-R7,LR}
  /*************************************   SET UP  *************************************************8*/

  
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

  LDR     R4, =GPIOE_ODR              
  LDR     R6, [R4]                    @ lightsOffState  
  LDR     R4, =FLASH_ON_TIMER         @
  MOV     R5, #500                    @ FLASH_ON_TIMER = 500;
  STR     R5, [R4]                    @
  LDR     R4, =reacted                @
  LDR     R5, [R4]                    @
  MOV     R5, #0                      @
  STR     R5, [R4]                    @                     


  LDR     R4, =GPIOE_MODER
  LDR     R5, =#0x55550000            @ configure all LEDs
  STR     R5,[R4]                     @
  MOV     R0, LD3_PIN                 @ currentPin = LD3_PIN

level1:                               @ do {
  LDR     R4, =reacted                @  
  LDR     R5, [R4]                    @
  CMP     R5, #1                      @ 
  BEQ     endLevel1                   @
  B       level1                      @ }while(reacted == False);
endLevel1:

  BL      nextLevel                   @ flash all LEDs

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 475;
  MOV     R5, #475                    @
  STR     R5, [R4]                    @
  LDR     R4, =reacted                @
  LDR     R5, [R4]                    @
  MOV     R5, #0                      @
  STR     R5, [R4]                    @
  MOV     R0, LD4_PIN                 @ currentPin = LD4_PIN

 level2:                              @ do {
  LDR     R4, =reacted                @
  LDR     R5, [R4]                    @
  CMP     R5, #1                      @
  BEQ     endLevel2                   @
  B       level2                      @ } while(reacted == False);
endLevel2:
  BL      nextLevel

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 450;
  MOV     R5, #450                    @
  STR     R5, [R4]                    @
  MOV     R0, LD6_PIN                 @ currentPin = LD6_PIN
  LDR     R4, =reacted                @
  MOV     R5,#0                       @
  STR     R5, [R4]                    @ reacted = False 

level3:
  LDR     R4, =reacted                @ do { 
  LDR     R5, [R4]                    @  
  CMP     R5, #1                      @ 
  BEQ     endLevel3                   @          
  B       level3                      @ }while(reacted == False);  
endLevel3:

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 425;
  MOV     R5, #425                    @
  STR     R5, [R4]                    @
  MOV     R0, LD8_PIN                 @ currentPin = LD8_PIN
  MOV     R5,#0                       @
  STR     R5, [R4]                    @ reacted = False 

level4:
  LDR     R4, = reacted                @ do {
  LDR     R5, [R4]                     @   
  CMP     R5, #1                       @
  BEQ     endLevel4                    @
  B       level4                       @ }while(reacted == False);
endLevel4:
  BL      nextLevel                   @ flash all LEDs

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 400;
  MOV     R5, #400                    @
  STR     R5, [R4]                    @
  MOV     R0, LD8_PIN                 @ currentPin = LD8_PIN
  LDR     R4, =reacted                @
  MOV     R5,#0                       @
  STR     R5, [R4]                    @ reacted = False  

level5:
  LDR     R4, =reacted                @ do {
  LDR     R5, [R4]                    @   
  CMP     R5, #1                      @   
  BEQ     endLevel5                   @                 
  B       level5                      @ }while(reacted == False);
endLevel5:
  BL      nextLevel                   @ flash all LEDs

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 375;
  MOV     R5, #375                    @
  STR     R5, [R4]                    @
  MOV     R0, LD10_PIN                @ currentPin = LD10_PIN
  LDR     R4, =reacted                @
  MOV     R5,#0                       @
  STR     R5, [R4]                    @ reacted = False  

level6:
  LDR     R4, =reacted                @ do {
  LDR     R5, [R4]                    @  
  CMP     R5, #1                      @
  BEQ     endLevel6                   @ 
  B       level6                      @ }while(reacted == False);
endLevel6:
  BL      nextLevel                   @ flash all LEDs

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 350;
  MOV     R5, #350                    @
  STR     R5, [R4]                    @
  MOV     R0, LD9_PIN                 @ currentPin = LD9_PIN
  LDR     R4, =reacted                @~
  MOV     R5, #0
  STR     R5, [R4]                    @ reacted = False  

level7:
  LDR     R4, =reacted                @ do {
  LDR     R5, [R4]                    @  
  CMP     R5, #1                      
  BEQ     endLevel7                   
  B       level7                      @ }while(reacted == False);
endLevel7:
  BL      nextLevel                   @ flash all LEDs

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 200;
  MOV     R5, #200
  STR     R5, [R4]
  MOV     R0, LD7_PIN                 @ currentPin = LD7_PIN
  LDR     R4, =reacted 
  MOV     R5,#0
  STR     R5, [R4]                    @ reacted = False  

level8:
  LDR     R4, =reacted                @ do {
  LDR     R5, [R4]                    @  
  CMP     R5, #1                      
  BEQ     endLevel8                   
  B       level8                      @ }while(reacted == False);
endLevel8:
  BL      nextLevel                   @ flash all LEDs

  LDR     R4, =FLASH_ON_TIMER         @ FLASH_ON_TIMER = 100;
  MOV     R5, #100
  STR     R5, [R4]
  MOV     R0, LD5_PIN                 @ currentPin = LD5_PIN
  LDR     R4, = reacted 
  MOV     R5,#0

  STR     R5, [R4]                    @ reacted = False  

level9:
  LDR     R4, =reacted                @ do {
  LDR     R5, [R4]                    @  
  CMP     R5, #1                      
  BEQ     endLevel9                   
  B       level9                      @  }while(reacted == False);
endLevel9:
  



// C. Quinn, created ending sequence

  // turning off all led's
  LDR     R4, =GPIOE_ODR
  STR     R6,[R4]                       @ turn off all the lights

lightSequence:
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD3_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write
  @ wait for 1s ...
  STR     R6,[R4]                       @ turn off all the lights
  LDR     R5, =500000                   @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL1:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL1  
//==== Lvl 2
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD4_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write
  @ wait for 1s ...
  LDR     R5, =500000                   @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL2:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL2  
//==== Lvl 3
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD6_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write
  @ wait for 1s ...
  LDR     R5, =500000                   @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL3:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL3  
//==== Lvl 4
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD8_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write
  @ wait for 1s ...
  LDR     R5, =500000                   @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL4:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL4  
//==== Lvl 5
 
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD10_PIN))        @ Modify ...
  STR     R5, [R4]                      @ Write
  @ wait for 1s ...
  LDR     R5, =500000                   @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL5:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL5  
  //==== Lvl 6
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD9_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write

  @ wait for 1s ...
  LDR     R5, =500000                   @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL6:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL6  
  //==== Lvl 7
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD7_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write
  @ wait for 1s ...
  LDR     R5, =500000                   @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL7:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL7  
  //==== Lvl 8
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD5_PIN))         @ Modify ...
  STR     R5, [R4]                      @ Write
  @ wait for 1s ...
   LDR     R5, =500000                  @ Assuming 8MHz clock, 4 cycles per iteration
                                        @ (SUBS + BNE + 2 stall cycles for branch)
.LwhwaitL8:
  SUBS    R5, R5, #1                    @ Keep looping until we count down to zero
  BNE     .LwhwaitL8  
  
  CMP     R7, #1
  BEQ     End_Main
  MOV     R7, #1            
  B       lightSequence
End_Main:
  POP     {R4-R7,PC}


/*******************************Subroutines & Interrupts **********************/

 
@
@ SysTick interrupt handler (blink currentPin)
@
@   parameter 
@     R0  - currentPin
  .type  SysTick_Handler, %function
SysTick_Handler:
  PUSH    {R0-R9, LR}                      
  LDR     R4, =blink_countdown          @ if (countdown != 0) {
  LDR     R5, [R4]                      @
  CMP     R5, #0                        @
  BEQ     .LelseFire                    @
  SUB     R5, R5, #1                    @   countdown = countdown - 1;
  STR     R5, [R4]                      @
  B       .LendIfDelay                  @ }
.LelseFire:                             @ else {
  LDR     R6, = reacted                 @
  LDR     R7,  [R6]
  CMP     R7,#1                         @   )
  BEQ     .LskipInvert                  @   if(reacted) 
  LDR     R4, =GPIOE_ODR                @   {
  LDR     R5, [R4]                      @   ledStatus = read(GPIOE_ODR)
  MOV     R6, #0b1                      
  MOV     R6, R6, LSL R0
  EOR     R5,R6                         @   GPIOE_ODR = GPIOE_ODR ^ (1<<currentPin);
  STR     R5, [R4]                      @   ledStatus = !ledStatus
.LskipInvert:                           @   }
  MOV     R6, #0b1
  MOV     R6, R6, LSL R0
  AND     R5, R6
  CMP     R5,R6                         @ if(ledStatus == off)
  BEQ     .LflashOn                     @   {
  LDR     R4, =blink_countdown          @     
  LDR     R5, = FLASH_OFF_TIMER         @
  STR     R5, [R4]                      @     countdown = FLASH_OFF_TIMER;
  B       .LendIfDelay                  @   }
.LflashOn:
  LDR     R4, =blink_countdown          @   else{
  LDR     R5, = FLASH_ON_TIMER
  LDR     R5, [R5]
  STR     R5, [R4]                      @   countdown = FLASH_ON_TIMER;
.LendIfDelay:                           @     }
  LDR     R4, =SCB_ICSR                 @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR       @
  STR     R5, [R4]                      @
  POP  {R0-R9, PC}                      @ Return from interrupt handler


@
@ External interrupt line 0 interrupt handler(Push Button Handler)
@   (Check if player pressed the button on time)
@
@     parameters
@       R0 - currentPin
  .type  EXTI0_IRQHandler, %function

EXTI0_IRQHandler:                                 
  PUSH    {R4-R6,LR}                    @ if(buttonPressed)
  LDR     R4, =GPIOE_ODR                @   {
  LDR     R5, [R4]                      @   ledStatus = read(GPIOE_ODR) 
  MOV     R6, #0b1                      @   
  MOV     R6, R6, LSL R0                @   
  AND     R5, R6                        @   if(ledStatus == On)
  CMP     R5, R6                        @     {
  BNE     .LledNotOn                    @       
  LDR     R4, =reacted                  @         reacted = true;
  MOV     R5, #1                        @     }
  STR     R5, [R4]                      @   
  .LledNotOn:                           @   }
  LDR     R4, =EXTI_PR                    
  MOV     R5, #(1<<0)                     
  STR     R5, [R4]                        
  POP     {R4-R6,PC}                      
  
  @ Next Level subroutine
  @   flashes the remaining levels
  @
nextLevel:
  PUSH    {R0, R5-R7, LR}                 @ Save registers and return address
  MOV     R6, #8                          @ Initialize loop counter to 0
  MOV     R5, #8
  MOV     R7, #0

.LturnOnLoop:

  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD3_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD4_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD5_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD6_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD7_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD8_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write
   LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD9_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4]                      @ Read ...
  EOR     R5, #(0b1<<(LD10_PIN))         @ Modify ..
  STR     R5, [R4]                      @ Write
  
  LDR R5, =1000000                                     @ (SUBS + BNE + 2 stall cycles for branch)
.Lwhwait:
  SUBS    R5, R5, #1                     @ Keep looping until we count down to zero
  BNE     .Lwhwait 
  cmp      R7, #1
  BEQ      finish
  Mov     R7,  #1
  B       .LturnOnLoop
finish:
  POP     {R0, R5-R7, PC}                 @ Restore registers and return

// memoryAddresses
  .section .data
blink_countdown:
  .space  4
reacted:
  .space 4
FLASH_ON_TIMER:
  .space 4



  .end