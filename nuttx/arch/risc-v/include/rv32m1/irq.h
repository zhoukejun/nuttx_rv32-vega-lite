/************************************************************************************
 * arch/risc-v/include/rv32m1/irq.h
 * RV32M1 event system
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
 ************************************************************************************/
/************************************************************************************
 *  RV32M1 features a FC controller and a 8-core cluster. IRQ from peripherals have
 *  unique ID, which are dispatched to the FC or cluster by the SOC event unit, and
 *  then by the FC event unit or cluster event unit, and finally to FC or cluster.
 *  Peripherals share the same IRQ entry.
 ************************************************************************************/

#ifndef __ARCH_RISC_V_INCLUDE_RV32M1_IRQ_H
#define __ARCH_RISC_V_INCLUDE_RV32M1_IRQ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <arch/irq.h>
#include <stdint.h>

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/

/** Interrupt Number Definitions */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Device specific interrupts */
  DMA0_0_4_8_12_IRQn           = 0,                /**< DMA0 channel 0/4/8/12 transfer complete */
  DMA0_1_5_9_13_IRQn           = 1,                /**< DMA0 channel 1/5/9/13 transfer complete */
  DMA0_2_6_10_14_IRQn          = 2,                /**< DMA0 channel 2/6/10/14 transfer complete */
  DMA0_3_7_11_15_IRQn          = 3,                /**< DMA0 channel 3/7/11/15 transfer complete */
  DMA0_Error_IRQn              = 4,                /**< DMA0 channel 0-15 error interrupt */
  CMC0_IRQn                    = 5,                /**< Core Mode Controller 0 */
  MUA_IRQn                     = 6,                /**< MU Side A interrupt */
  USB0_IRQn                    = 7,                /**< USB0 interrupt */
  USDHC0_IRQn                  = 8,                /**< SDHC0 interrupt */
  I2S0_IRQn                    = 9,                /**< I2S0 interrupt */
  FLEXIO0_IRQn                 = 10,               /**< FLEXIO0 */
  EMVSIM0_IRQn                 = 11,               /**< EMVSIM0 interrupt */
  LPIT0_IRQn                   = 12,               /**< LPIT0 interrupt */
  LPSPI0_IRQn                  = 13,               /**< LPSPI0 single interrupt vector for all sources */
  LPSPI1_IRQn                  = 14,               /**< LPSPI1 single interrupt vector for all sources */
  LPI2C0_IRQn                  = 15,               /**< LPI2C0 interrupt */
  LPI2C1_IRQn                  = 16,               /**< LPI2C1 interrupt */
  LPUART0_IRQn                 = 17,               /**< LPUART0 status and error */
  PORTA_IRQn                   = 18,               /**< PORTA Pin detect */
  TPM0_IRQn                    = 19,               /**< TPM0 single interrupt vector for all sources */
  ADC0_IRQn                    = 21,               /**< LPADC0 interrupt */
  LPDAC0_IRQn                  = 20,               /**< DAC0 interrupt */
  LPCMP0_IRQn                  = 22,               /**< LPCMP0 interrupt */
  RTC_IRQn                     = 23,               /**< RTC Alarm interrupt */
  INTMUX0_0_IRQn               = 24,               /**< INTMUX0 channel0 interrupt */
  INTMUX0_1_IRQn               = 25,               /**< INTMUX0 channel1 interrupt */
  INTMUX0_2_IRQn               = 26,               /**< INTMUX0 channel2 interrupt */
  INTMUX0_3_IRQn               = 27,               /**< INTMUX0 channel3 interrupt */
  INTMUX0_4_IRQn               = 28,               /**< INTMUX0 channel4 interrupt */
  INTMUX0_5_IRQn               = 29,               /**< INTMUX0 channel5 interrupt */
  INTMUX0_6_IRQn               = 30,               /**< INTMUX0 channel6 interrupt */
  INTMUX0_7_IRQn               = 31,               /**< INTMUX0 channel7 interrupt */
  EWM_IRQn                     = 32,               /**< EWM interrupt */
  FTFE_Command_Complete_IRQn   = 33,               /**< FTFE interrupt */
  FTFE_Read_Collision_IRQn     = 34,               /**< FTFE interrupt */
  LLWU0_IRQn                   = 35,               /**< Low leakage wakeup 0 */
  SPM_IRQn                     = 36,               /**< SPM */
  WDOG0_IRQn                   = 37,               /**< WDOG0 interrupt */
  SCG_IRQn                     = 38,               /**< SCG interrupt */
  LPTMR0_IRQn                  = 39,               /**< LPTMR0 interrupt */
  LPTMR1_IRQn                  = 40,               /**< LPTMR1 interrupt */
  TPM1_IRQn                    = 41,               /**< TPM1 single interrupt vector for all sources */
  TPM2_IRQn                    = 42,               /**< TPM2 single interrupt vector for all sources */
  LPI2C2_IRQn                  = 43,               /**< LPI2C2 interrupt */
  LPSPI2_IRQn                  = 44,               /**< LPSPI2 single interrupt vector for all sources */
  LPUART1_IRQn                 = 45,               /**< LPUART1 status and error */
  LPUART2_IRQn                 = 46,               /**< LPUART2 status and error */
  PORTB_IRQn                   = 47,               /**< PORTB Pin detect */
  PORTC_IRQn                   = 48,               /**< PORTC Pin detect */
  PORTD_IRQn                   = 49,               /**< PORTD Pin detect */
  CAU3_Task_Complete_IRQn      = 50,               /**< Cryptographic Acceleration Unit version 3 Task Complete */
  CAU3_Security_Violation_IRQn = 51,               /**< Cryptographic Acceleration Unit version 3 Security Violation */
  TRNG_IRQn                    = 52,               /**< TRNG interrupt */
  LPIT1_IRQn                   = 53,               /**< LPIT1 interrupt */
  LPTMR2_IRQn                  = 54,               /**< LPTMR2 interrupt */
  TPM3_IRQn                    = 55,               /**< TPM3 single interrupt vector for all sources */
  LPI2C3_IRQn                  = 56,               /**< LPI2C3 interrupt */
  LPSPI3_IRQn                  = 57,               /**< LPSPI3 single interrupt vector for all sources */
  LPUART3_IRQn                 = 58,               /**< LPUART3 status and error */
  PORTE_IRQn                   = 59,               /**< PORTE Pin detect */
  LPCMP1_IRQn                  = 60,               /**< LPCMP1 interrupt */
  RF0_0_IRQn                   = 61,               /**< RF0 interrupt 0 */
  RF0_1_IRQn                   = 62,               /**< RF0 interrupt 1 */
} IRQn_Type;

#define  NR_IRQS		62

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupt and return the current interrupt state.
 *
 ****************************************************************************/

static inline uint32_t up_irq_save(void)
{
  uint32_t oldstat, newstat;

  asm volatile ("csrr %0, 0x300": "=r" (oldstat));
  newstat = oldstat & ~(0x9);

  asm volatile("csrw 0x300, %0" : /* no output */ : "r" (newstat));

  return oldstat;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous IRQ mask state
 *
 ****************************************************************************/

static inline void up_irq_restore(uint32_t pri)
{
      asm volatile("csrw 0x300, %0" : /* no output */ : "r" (pri));
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

static inline uint32_t up_irq_enable(void)
{
  uint32_t oldstat, newstat;

  asm volatile ("csrr %0, 0x300": "=r" (oldstat));
  newstat = oldstat | (0x9);

  asm volatile("csrw 0x300, %0" : /* no output */ : "r" (newstat));

  return oldstat;
}

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_RISC_V_INCLUDE_RV32M1_IRQ_H */
