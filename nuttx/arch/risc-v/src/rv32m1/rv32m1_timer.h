/************************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_timer.h
 * Timer driver for RV32M1
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
 ************************************************************************************/


#ifndef __ARCH_RISC_V_SRC_GAP8_TIM_H
#define __ARCH_RISC_V_SRC_GAP8_TIM_H

//TODO: ZK
/* Use LIPT0 channel 0 for systick. */
#define SYSTICK_LPIT LPIT0
#define SYSTICK_LPIT_CH 0
#define SYSTICK_LPIT_IRQn LPIT0_IRQn

/************************************************************************************
 * Included Files
 ************************************************************************************/


/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: rv32m1_timer_initialize
 *
 * Description:
 *   Initialize the timer based on the frequency of source clock and ticks
 *   per second.
 *
 ****************************************************************************/

void rv32m1_timer_initialize(uint32_t source_clock, uint32_t tick_per_second);

/****************************************************************************
 * Name: rv32m1_register_timercallback
 *
 * Description:
 *   Register a callback function called on timer IRQ
 *
 ****************************************************************************/

void rv32m1_register_timercallback(void (*on_timer)(void*arg), void *arg);


#endif /* __ARCH_RISC_V_SRC_RV32M1_TIMER_H */
