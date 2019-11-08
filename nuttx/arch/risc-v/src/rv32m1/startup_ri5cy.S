/****************************************************************************
 * arch/risc-v/src/gapuino/startup_ri5cy.S
 * Startup file for RI5CY Core of RV32M1
 *  Interrupt vector and reset handler
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: zhoukejun <zhoukejun@outlook.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Exception context size: EPC + 31 common regs + 6 loop regs */

#define EXCEPTION_STACK_SIZE 4*38

/****************************************************************************
 * Assembler Macro Definitions
 ****************************************************************************/

/* save all the registers on interrupt entry */

  .macro SAVE_REGS
    addi sp, sp, -EXCEPTION_STACK_SIZE
    sw  x1,  1*4(sp)  /* ra */
    sw  x3,  3*4(sp)  /* gp */
    sw  x4,  4*4(sp)  /* tp */
    sw  x5,  5*4(sp)  /* t0 */
    sw  x6,  6*4(sp)  /* t1 */
    sw  x7,  7*4(sp)  /* t2 */
    sw  x8,  8*4(sp)  /* s0 */
    sw  x9,  9*4(sp)  /* s1 */
    sw x10, 10*4(sp)  /* a0 */
    sw x11, 11*4(sp)  /* a1 */
    sw x12, 12*4(sp)  /* a2 */
    sw x13, 13*4(sp)  /* a3 */
    sw x14, 14*4(sp)  /* a4 */
    sw x15, 15*4(sp)  /* a5 */
    sw x16, 16*4(sp)  /* a6 */
    sw x17, 17*4(sp)  /* a7 */
    sw x18, 18*4(sp)  /* s2 */
    sw x19, 19*4(sp)  /* s3 */
    sw x20, 20*4(sp)  /* s4 */
    sw x21, 21*4(sp)  /* s5 */
    sw x22, 22*4(sp)  /* s6 */
    sw x23, 23*4(sp)  /* s7 */
    sw x24, 24*4(sp)  /* s8 */
    sw x25, 25*4(sp)  /* s9 */
    sw x26, 26*4(sp)  /* s10 */
    sw x27, 27*4(sp)  /* s11 */
    sw x28, 28*4(sp)  /* t3 */
    sw x29, 29*4(sp)  /* t4 */
    sw x30, 30*4(sp)  /* t5 */
    sw x31, 31*4(sp)  /* t6 */
    csrr x28, 0x7B0
    csrr x29, 0x7B1
    csrr x30, 0x7B2
    sw x28, 32*4(sp)  /* lpstart[0] */
    sw x29, 33*4(sp)  /* lpend[0] */
    sw x30, 34*4(sp)  /* lpcount[0] */
    csrr x28, 0x7B4
    csrr x29, 0x7B5
    csrr x30, 0x7B6
    sw x28, 35*4(sp)  /* lpstart[1] */
    sw x29, 36*4(sp)  /* lpend[1] */
    sw x30, 37*4(sp)  /* lpcount[1] */
    addi s0, sp, EXCEPTION_STACK_SIZE
    sw  s0,  2*4(sp)   /* original SP */
  .endm

/* restore regs and `mret` */

  .macro RESTORE_REGS
    lw x28, 35*4(sp)  /* lpstart[1] */
    lw x29, 36*4(sp)  /* lpend[1] */
    lw x30, 37*4(sp)  /* lpcount[1] */
    csrrw x0, 0x7B4, x28
    csrrw x0, 0x7B5, x29
    csrrw x0, 0x7B6, x30
    lw x28, 32*4(sp)  /* lpstart[0] */
    lw x29, 33*4(sp)  /* lpend[0] */
    lw x30, 34*4(sp)  /* lpcount[0] */
    csrrw x0, 0x7B0, x28
    csrrw x0, 0x7B1, x29
    csrrw x0, 0x7B2, x30
    li s0, 0x1890     /* machine mode, UPIE & MPIE enabled */
    csrrw x0, mstatus, s0
    lw  x3,  3*4(sp)  /* gp */
    lw  x4,  4*4(sp)  /* tp */
    lw  x5,  5*4(sp)  /* t0 */
    lw  x6,  6*4(sp)  /* t1 */
    lw  x7,  7*4(sp)  /* t2 */
    lw  x8,  8*4(sp)  /* s0 */
    lw  x9,  9*4(sp)  /* s1 */
    lw x10, 10*4(sp)  /* a0 */
    lw x11, 11*4(sp)  /* a1 */
    lw x12, 12*4(sp)  /* a2 */
    lw x13, 13*4(sp)  /* a3 */
    lw x14, 14*4(sp)  /* a4 */
    lw x15, 15*4(sp)  /* a5 */
    lw x16, 16*4(sp)  /* a6 */
    lw x17, 17*4(sp)  /* a7 */
    lw x18, 18*4(sp)  /* s2 */
    lw x19, 19*4(sp)  /* s3 */
    lw x20, 20*4(sp)  /* s4 */
    lw x21, 21*4(sp)  /* s5 */
    lw x22, 22*4(sp)  /* s6 */
    lw x23, 23*4(sp)  /* s7 */
    lw x24, 24*4(sp)  /* s8 */
    lw x25, 25*4(sp)  /* s9 */
    lw x26, 26*4(sp)  /* s10 */
    lw x27, 27*4(sp)  /* s11 */
    lw x28, 28*4(sp)  /* t3 */
    lw x29, 29*4(sp)  /* t4 */
    lw x30, 30*4(sp)  /* t5 */
    lw x31, 31*4(sp)  /* t6 */

    lw  x1,  1*4(sp)  /* ra */

    lw  sp,  2*4(sp)  /* restore original sp */
  .endm

/* wrapper for IRQ vector */

  .macro WRAP_IRQ Routine, IRQn
  wrap_irq_\Routine :
    SAVE_REGS

    csrr s0, mepc
    sw  s0,  0(sp)   /* exception PC */

    li a0, \IRQn     /* irq = IRQn */
    mv a1, sp        /* context = sp */
    jal x1, rv32m1_dispatch_irq

    /* If context switch is needed, return
     * a new sp
     */

    mv sp, a0

    lw  s0, 0(sp)    /* restore ePC */
    csrw mepc, s0

    RESTORE_REGS

    mret
  .endm


/*******************************************************************************
 * External Variables and Functions
 *******************************************************************************/

  .extern _sbss
  .extern _ebss
  .extern _idle_stack_end

  .extern rv32m1_dispatch_irq
  .extern nx_start
  .extern SystemInit

/*******************************************************************************
 * Reset handler
 *******************************************************************************/

    .global __start
    .section .startup
__start:
reset_handler:

  /* Disable global interrupt. */
  csrci mstatus, 8

  /* Initialize stack pointer */
  la sp, _stack_start

  /* Initialize global pointer */
  la gp, __global_pointer

  la  x2, _idle_stack_end

  /* Clear BSS */

  la x26, _sbss
  la x27, _ebss

  bge x26, x27, zero_loop_end

zero_loop:
  sw x0, 0(x26)
  addi x26, x26, 4
  ble x26, x27, zero_loop

zero_loop_end:


  /* TODO: initialize data section */

  /* Initialize cache and clock */

  jal x1, SystemInit 

  /* Enable global interrupt. */
  csrsi mstatus, 8

  jal up_lowsetup

  /* Directly call Nuttx nx_start() */

  jal x1, nx_start

  /* If it ever returns, spin here forever... */

dead_loop:
  jal x0, dead_loop


/* IRQ wrappers
 *  IRQn are identical to rv32m1_irq.h
 */


  /* Device specific interrupts */
WRAP_IRQ DMA0_0_4_8_12_IRQn,            0                /**< DMA0 channel 0/4/8/12 transfer complete */
WRAP_IRQ DMA0_1_5_9_13_IRQn,            1                /**< DMA0 channel 1/5/9/13 transfer complete */
WRAP_IRQ DMA0_2_6_10_14_IRQn,           2                /**< DMA0 channel 2/6/10/14 transfer complete */
WRAP_IRQ DMA0_3_7_11_15_IRQn,           3                /**< DMA0 channel 3/7/11/15 transfer complete */
WRAP_IRQ DMA0_Error_IRQn,               4                /**< DMA0 channel 0-15 error interrupt */
WRAP_IRQ CMC0_IRQn,                     5                /**< Core Mode Controller 0 */
WRAP_IRQ MUA_IRQn,                      6                /**< MU Side A interrupt */
WRAP_IRQ USB0_IRQn,                     7                /**< USB0 interrupt */
WRAP_IRQ USDHC0_IRQn,                   8                /**< SDHC0 interrupt */
WRAP_IRQ I2S0_IRQn,                     9                /**< I2S0 interrupt */
WRAP_IRQ FLEXIO0_IRQn,                  10               /**< FLEXIO0 */
WRAP_IRQ EMVSIM0_IRQn,                  11               /**< EMVSIM0 interrupt */
WRAP_IRQ LPIT0_IRQn,                    12               /**< LPIT0 interrupt */
WRAP_IRQ LPSPI0_IRQn,                   13               /**< LPSPI0 single interrupt vector for all sources */
WRAP_IRQ LPSPI1_IRQn,                   14               /**< LPSPI1 single interrupt vector for all sources */
WRAP_IRQ LPI2C0_IRQn,                   15               /**< LPI2C0 interrupt */
WRAP_IRQ LPI2C1_IRQn,                   16               /**< LPI2C1 interrupt */
WRAP_IRQ LPUART0_IRQn,                  17               /**< LPUART0 status and error */
WRAP_IRQ PORTA_IRQn,                    18               /**< PORTA Pin detect */
WRAP_IRQ TPM0_IRQn,                     19               /**< TPM0 single interrupt vector for all sources */
WRAP_IRQ ADC0_IRQn,                     21               /**< LPADC0 interrupt */
WRAP_IRQ LPDAC0_IRQn,                   20               /**< DAC0 interrupt */
WRAP_IRQ LPCMP0_IRQn,                   22               /**< LPCMP0 interrupt */
WRAP_IRQ RTC_IRQn,                      23               /**< RTC Alarm interrupt */
WRAP_IRQ INTMUX0_0_IRQn,                24               /**< INTMUX0 channel0 interrupt */
WRAP_IRQ INTMUX0_1_IRQn,                25               /**< INTMUX0 channel1 interrupt */
WRAP_IRQ INTMUX0_2_IRQn,                26               /**< INTMUX0 channel2 interrupt */
WRAP_IRQ INTMUX0_3_IRQn,                27               /**< INTMUX0 channel3 interrupt */
WRAP_IRQ INTMUX0_4_IRQn,                28               /**< INTMUX0 channel4 interrupt */
WRAP_IRQ INTMUX0_5_IRQn,                29               /**< INTMUX0 channel5 interrupt */
WRAP_IRQ INTMUX0_6_IRQn,                30               /**< INTMUX0 channel6 interrupt */
WRAP_IRQ INTMUX0_7_IRQn,                31               /**< INTMUX0 channel7 interrupt */
WRAP_IRQ EWM_IRQn,                      32               /**< EWM interrupt */
WRAP_IRQ FTFE_Command_Complete_IRQn,    33               /**< FTFE interrupt */
WRAP_IRQ FTFE_Read_Collision_IRQn,      34               /**< FTFE interrupt */
WRAP_IRQ LLWU0_IRQn,                    35               /**< Low leakage wakeup 0 */
WRAP_IRQ SPM_IRQn,                      36               /**< SPM */
WRAP_IRQ WDOG0_IRQn,                    37               /**< WDOG0 interrupt */
WRAP_IRQ SCG_IRQn,                      38               /**< SCG interrupt */
WRAP_IRQ LPTMR0_IRQn,                   39               /**< LPTMR0 interrupt */
WRAP_IRQ LPTMR1_IRQn,                   40               /**< LPTMR1 interrupt */
WRAP_IRQ TPM1_IRQn,                     41               /**< TPM1 single interrupt vector for all sources */
WRAP_IRQ TPM2_IRQn,                     42               /**< TPM2 single interrupt vector for all sources */
WRAP_IRQ LPI2C2_IRQn,                   43               /**< LPI2C2 interrupt */
WRAP_IRQ LPSPI2_IRQn,                   44               /**< LPSPI2 single interrupt vector for all sources */
WRAP_IRQ LPUART1_IRQn,                  45               /**< LPUART1 status and error */
WRAP_IRQ LPUART2_IRQn,                  46               /**< LPUART2 status and error */
WRAP_IRQ PORTB_IRQn,                    47               /**< PORTB Pin detect */
WRAP_IRQ PORTC_IRQn,                    48               /**< PORTC Pin detect */
WRAP_IRQ PORTD_IRQn,                    49               /**< PORTD Pin detect */
WRAP_IRQ CAU3_Task_Complete_IRQn,       50               /**< Cryptographic Acceleration Unit version 3 Task Complete */
WRAP_IRQ CAU3_Security_Violation_IRQn,  51              /**< Cryptographic Acceleration Unit version 3 Security Violation */
WRAP_IRQ TRNG_IRQn,                     52               /**< TRNG interrupt */
WRAP_IRQ LPIT1_IRQn,                    53               /**< LPIT1 interrupt */
WRAP_IRQ LPTMR2_IRQn,                   54               /**< LPTMR2 interrupt */
WRAP_IRQ TPM3_IRQn,                     55               /**< TPM3 single interrupt vector for all sources */
WRAP_IRQ LPI2C3_IRQn,                   56               /**< LPI2C3 interrupt */
WRAP_IRQ LPSPI3_IRQn,                   57               /**< LPSPI3 single interrupt vector for all sources */
WRAP_IRQ LPUART3_IRQn,                  58               /**< LPUART3 status and error */
WRAP_IRQ PORTE_IRQn,                    59               /**< PORTE Pin detect */
WRAP_IRQ LPCMP1_IRQn,                   60               /**< LPCMP1 interrupt */
WRAP_IRQ RF0_0_IRQn,                    61               /**< RF0 interrupt 0 */
WRAP_IRQ RF0_1_IRQn,                    62               /**< RF0 interrupt 1 */
/* RISCV exceptions */

illegal_insn_handler:
  csrr s0, mepc
  sw  s0, 0*4(sp)   /* exception PC */

  /* Spin here so that debugger would read `s0` */

1:
  j 1b

/* Systemcall handler */

ecall_insn_handler:
  SAVE_REGS

  /* Point to the next instruction of `ecall` */

  csrr s0, mepc
  addi s0, s0, 4
  sw  s0,  0(sp)   /* exception PC */

  li a0, 34        /* irq = 34 */
  mv a1, sp        /* context = sp */
  jal x1, rv32m1_dispatch_irq

  /* If context switch is needed, return
   * a new sp
   */

  mv sp, a0

  lw  s0, 0(sp)    /* restore ePC */
  csrw mepc, s0

  RESTORE_REGS

  mret

/*******************************************************************************
 *  INTERRUPT VECTOR TABLE
 *******************************************************************************/

  /* This section has to be down here, since we have to disable rvc for it  */

  .section .vectors, "ax"
  .option norvc;

  jal x0, IRQ_Handler /* 0 */
  jal x0, IRQ_Handler /* 1 */
  jal x0, IRQ_Handler /* 2 */
  jal x0, IRQ_Handler /* 3 */
  jal x0, IRQ_Handler /* 4 */
  jal x0, IRQ_Handler /* 5 */
  jal x0, IRQ_Handler /* 6 */
  jal x0, IRQ_Handler /* 7 */
  jal x0, IRQ_Handler /* 8 */
  jal x0, IRQ_Handler /* 9 */
  jal x0, IRQ_Handler /* 10 */
  jal x0, IRQ_Handler /* 11 */
  jal x0, IRQ_Handler /* 12 */
  jal x0, IRQ_Handler /* 13 */
  jal x0, IRQ_Handler /* 14 */
  jal x0, IRQ_Handler /* 15 */
  jal x0, IRQ_Handler /* 16 */
  jal x0, IRQ_Handler /* 17 */
  jal x0, IRQ_Handler /* 18 */
  jal x0, IRQ_Handler /* 19 */
  jal x0, IRQ_Handler /* 20 */
  jal x0, IRQ_Handler /* 21 */
  jal x0, IRQ_Handler /* 22 */
  jal x0, IRQ_Handler /* 23 */
  jal x0, IRQ_Handler /* 24 */
  jal x0, IRQ_Handler /* 25 */
  jal x0, IRQ_Handler /* 26 */
  jal x0, IRQ_Handler /* 27 */
  jal x0, IRQ_Handler /* 28 */
  jal x0, IRQ_Handler /* 29 */
  jal x0, IRQ_Handler /* 30 */
  jal x0, IRQ_Handler /* 31 */

  j reset_handler       /* 32 */
  j illegal_insn_handler/* 33 */
  j ecall_insn_handler  /* 34 */

/****************************************************************************
 * This variable is pointed to the structure containing all information
 * exchanged with the platform loader. It is using a fixed address so that
 * the loader can also find it and then knows the address of the debug
 * structure.
 ****************************************************************************/

  .section .dbg_struct, "ax"
  .option norvc;
  .org 0x90
  .global __rt_debug_struct_ptr
__rt_debug_struct_ptr:
  .word Debug_Struct

    // load back registers from stack
end_except:
    lw a1, 0x50(x2)  // mstatus
    csrrw x0, 0x300, a1
    lw a0, 0x4c(x2)  // mepc
    csrrw x0, 0x341, a0

    lw a0, 0x40(x2)  // lpstart[1]
    lw a1, 0x44(x2)  // lpend[1]
    lw a2, 0x48(x2)  // lpcount[1]
    csrrw x0, 0x7B4, a0
    csrrw x0, 0x7B5, a1
    csrrw x0, 0x7B6, a2
    lw a0, 0x34(x2)  // lpstart[0]
    lw a1, 0x38(x2)  // lpend[0]
    lw a2, 0x3c(x2)  // lpcount[0]
    csrrw x0, 0x7B0, a0
    csrrw x0, 0x7B1, a1
    csrrw x0, 0x7B2, a2

    lw  x3, 0x00(x2)  // gp
    lw  x4, 0x04(x2)  // tp
    lw  x5, 0x08(x2)  // t0
    lw  x6, 0x0c(x2)  // t1
    lw  x7, 0x10(x2)  // t2
    lw x10, 0x14(x2)  // a0
    lw x11, 0x18(x2)  // a1
    lw x12, 0x1c(x2)  // a2
    lw x13, 0x20(x2)  // a3
    lw x14, 0x24(x2)  // a4
    lw x15, 0x28(x2)  // a5
    lw x16, 0x2c(x2)  // a6
    lw x17, 0x30(x2)  // a7

    lw  x1, 0x54(x2)
    addi x2, x2, EXCEPTION_STACK_SIZE
    mret

  .weak IRQ_Handler
  .type IRQ_Handler, %function
IRQ_Handler:
  addi x2, x2, -EXCEPTION_STACK_SIZE
  sw x1, 0x54(x2)
  SAVE_REGS
  la x1, end_except
  csrr a0, mcause
  jal x0, SystemIrqHandler
  .size IRQ_Handler, . - IRQ_Handler
/****************************************************************************
 * This global variable is unsigned int g_idle_topstack and is exported here
 * only because of its coupling to idle thread stack.
 ****************************************************************************/

  .section .data
  .global  g_idle_topstack
g_idle_topstack:
  .word  _idle_stack_end
