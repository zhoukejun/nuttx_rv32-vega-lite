/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_timer.c
 * GAP8 basic timer
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Kejun ZHOU <zhoukejun@outlook.com>
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

/************************************************************************************
 *  FC core has a 64-bit basic timer, able to split into 2 32-bit timers, with
 *  identicle memory map and 2 IRQ channels, for both FC core and cluster. We would
 *  use it as system timer.
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/chip/irq.h>

#include "rv32m1_timer.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rv32m1_timer_isr
 *
 * Description:
 *   Timer ISR to perform RR context switch
 *
 ****************************************************************************/

static int rv32m1_timer_isr(int irq, void *context, FAR void *arg)
{
  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_timer_initialize
 *
 * Description:
 *   Initialize the timer based on the frequency of source clock and ticks
 *   per second.
 *
 ****************************************************************************/

void riscv_timer_initialize(void)
{
  irq_attach(SYSTICK_LPIT_IRQn, rv32m1_timer_isr, NULL);

 // up_enable_irq(SYSTICK_LPIT_IRQn);
  SystemSetupSystick(1, 1);
}
