@ This is a program to use an interrupt-enabled timer to trigger an accumulating
@ 0.5 s count and display it in binary on the LEDs
@ this program reuses code provided by ARM for enabling interrupts

@ first set up exception vector table
.section .vectors, "ax"
  b _start
  b evt_undef
  b evt_undef
  b evt_undef
  b evt_undef
  .word 0
  b service_irq
  b evt_undef

.text
.global _start
_start:
  @ bit pattern to change to IRQ mode
  mov r1, #0b11010010
  @ change to IRQ mode
  msr cpsr_c, r1
  @ set up stack pointer for IRQ mode
  ldr sp, =0xFFFFFFFC

  @ bit pattern to change to supervisor mode
  @ with interrupts disabled
  mov r1, #0b11010011
  @ change to supervisor mode
  msr cpsr, r1
  @ set stack pointer for supervisor mode
  ldr sp, =0x3FFFFFFC

  @ call routine to configure the GIC
  bl config_gic

  @ initialize timer for 0.5 s count with interrupts
  @ timer address
  ldr r0, =0xFF202000
  @ turn off timer
  mov r1, #8
  str r1, [r0, #4]
  @ clear time-outs
  str r1, [r0]
  @ set interval
  ldr r1, =50000000
  @ write low period
  str r1, [r0, #8]
  @ shift bits right by 16
  lsr r1, #16
  @ write high period
  str r1, [r0, #12]
  @ bit pattern for count-down repeat with 
  @ interrupts for control register
  mov r1, #7
  @ start timer
  str r1, [r0, #4]

  @ turn on button interrupts
  @ there is an interrupt control register
  @ at [base+8]
  @ set bit(n) = 1 for button(n) to have 
  @ interrupts
  ldr r0, =0xFF200050
  @ here I just want button(0) to have interrupts
  @ so use bit(0) = 1
  @ use #15 (0b1111) to enable interrupts on
  @ all 4 buttons, etc.
  mov r1, #1
  str r1, [r0, #8]

  @ enable IRQ interrupts in the processor
  mov r0, #0b01010011
  msr cpsr_c, r0

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ main program is a dead loop
idle:
  b idle

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Define the exception service routines
@ most of these shouldn't happen, so they all
@ go to the same dead loop
evt_undef:
  b evt_undef

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ this one is for interrupts
service_irq:
  push {r0-r7, lr}

  @ read the ICCIAR from the CPU Interface
  ldr r4, =0xFFFEC100
  ldr r5, [r4, #12] 

  @ see if timer (IRQ #72)
  @ called interrupt
  cmp r5, #72
  beq timer_isr

  @ see if buttons (IRQ #73)
  @ called interrupt
  cmp r5, #73
  beq button_isr

  @ repeat the above for any additional
  @ hardware that uses interrupts

exit_irq:
  @ clear end-of-interrupt register (ICCEOIR)
  str r5, [r4, #16]

  pop {r0-r7, lr}
  @ return from IRQ mode
  subs pc, lr, #4

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ each time counter rolls over this routine is called
.global timer_isr
timer_isr:
  @ note that this is called from service_irq
  @ and service_irq already preserved state

  @ timer base address
  ldr r0, =0xFF202000
  @ clear timeout
  mov r1, #1
  str r1, [r0]

  @ LED address
  ldr r0, =0xFF200000
  @ get current LED pattern
  ldr r1, [r0]
  @ add 1 to pattern
  add r1, #1
  @ write new pattern to LED
  str r1, [r0]

  b exit_irq

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ each time button is pressed this routine is called
.global button_isr
button_isr:
  @ note that this is called from service_irq
  @ and service_irq already preserved state

  @ buttons base address
  ldr r0, =0xFF200050
  @ clear interrupt
  mov r1, #1
  str r1, [r0, #12]

  @ LED address
  ldr r0, =0xFF200000
  @ clear LED display
  mov r1, #0
  str r1, [r0]

  b exit_irq

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Configure the Generic Interrupt Controller (GIC)
.global config_gic
config_gic:
  push {lr}
  @ to configure an interrupt
  @  1. set target cpu in ICDIPTRn
  @  2. enable interrupt in ICDISERn

  @ timer is IRQ #72
  mov r0, #72
  @ bit mask for cpu0
  mov r1, #1 
  @ subroutine to configure GIC interrupt registers
  bl config_interrupt

  @ buttons are IRQ #73
  mov r0, #73
  @ bit mask for cpu0
  mov r1, #1 
  @ subroutine to configure GIC interrupt registers
  bl config_interrupt

  @ repeat the above for any additional
  @ hardware that uses interrupts

  @ configure the GIC CPU Interface
  @ base address of CPU Interface 
  ldr r0, =0xFFFEC100
  @ enable all priority levels
  ldr r1, =0xFFFF
  @ write to priority register (ICCPMR)
  str r1, [r0, #4]
  @ set the enable bit in the CPU Interface Control Register (ICCICR).
  mov r1, #1
  str r1, [r0]

  @ set the enable bit in the Distributor Control Register (ICDDCR).
  ldr r0, =0xFFFED000
  str r1, [r0]

  pop {lr}  
  bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ configure registers in GIC
@ this works for any IRQ
@ make sure IRQ # is in r0
@ and CPU target is in r1
@ usually r1 is #1
@
@ call this routine once for each IRQ #
config_interrupt:
  push {r4-r5, lr}

  @ calculate register in ICDISERn
  lsr r4, r0, #3
  bic r4, r4, #3
  @ get ICDISERn base address
  ldr r2, =0xFFFED100
  @ get IRQ #-specific ICDISERn address
  add r4, r2, r4
  @ find bit
  and r2, r0, #0x1F
  @ get bit mask to enable
  mov r5, #1 
  lsl r2, r5, r2
  @ now register address is in r4 and bit mask is in r2
  @ set the enable bit for specific IRQ # in ICDISERn register
  ldr r3, [r4]
  orr r3, r3, r2
  str r3, [r4]

  @ if you wanted to specify whether interrupt was
  @ edge- or level-triggered, you could do so here
  @ this uses ICDICFRn register
  @ note that this requires you to calculate different
  @ register and byte address based on IRQ
  @ I *think* this is the same as doubling the above addresses
  @ so:
  @  lsr r4, r0, #3
  @  bic r4, r4, #3
  @  lsl r4, #1
  @  ldr r2, =0xFFFEDC00
  @  add r4, r2, r4
  @  and r2, r0, #0x1F
  @  mov r5, #1
  @  lsl r2, r5, r2
  @  ldr r3, [r4]
  @  orr r3, r3, rn
  @  str r3, [r4]
  @ should work, but I am too lazy to check
  @ note that choice to set edge-/level-
  @ should be in rn (should probably be r2, but 
  @ then you need to change the use of r3 in all 
  @ the above code)

  @ now calculate register in ICDIPTRn
  bic r4, r0, #3
  @ get ICDIPTRn base address
  ldr r2, =0xFFFED800
  @ find IRQ #-specific ICDIPTRn address
  add r4, r2, r4
  @ find byte
  and r2, r0, #3
  @ combine word and byte address
  add r4, r2, r4
  @ now byte address is in r4
  @ set CPU target in byte for specific IRQ # in ICDIPTRn register
  strb r1, [r4]

  @ if you wanted to set a specific priority
  @ for the interrupt, you could do so here
  @ the priority register is ICDIPRn and has same
  @ address and byte as the CPU target
  @ so: 
  @  ldr r2, =0xFFFED400
  @  strb rx, [r4]
  @ works if rx is whatever register holds the priority
  @ value (should probably be r3, but then you need
  @ to change the use of r3 in all the above code)

  pop {r4-r5, lr}
  bx lr

.end