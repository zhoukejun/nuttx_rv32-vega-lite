/****************************************************************************
 * arch/risc-v/src/common/up_initialize.c
 *
 *   Copyright (C) 2007-2010, 2012-2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/mm/iob.h>
#include <nuttx/serial/pty.h>
#include <nuttx/syslog/syslog.h>
#include <nuttx/drivers/drivers.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 3
static inline void up_color_intstack(void)
{
  uint32_t *ptr = (uint32_t *)&g_intstackalloc;
  ssize_t size;

  for (size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
       size > 0;
       size -= sizeof(uint32_t))
    {
      *ptr++ = INTSTACK_COLOR;
    }
}
#else
#  define up_color_intstack()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.  The architecture specific
 *   details of initializing the OS will be handled here.  Such things as
 *   setting up interrupt service routines, starting the clock, and
 *   registering device drivers are some of the things that are different
 *   for each processor and hardware platform.
 *
 *   up_initialize is called after the OS initialized but before the user
 *   initialization logic has been started and before the libraries have
 *   been initialized.  OS services and driver services are available.
 *
 ****************************************************************************/

void up_initialize(void)
{
  /* Colorize the interrupt stack */

  up_color_intstack();

  /* Add any extra memory fragments to the memory manager */

  up_addregion();

  /* Initialize the interrupt subsystem */

  up_irqinitialize();

  /* Initialize the system timer interrupt */

#if !defined(CONFIG_SUPPRESS_INTERRUPTS) && !defined(CONFIG_SUPPRESS_TIMER_INTS) && \
    !defined(CONFIG_SYSTEMTICK_EXTCLK)
//  riscv_timer_initialize();
#endif

#ifdef CONFIG_MM_IOB
  /* Initialize IO buffering */

  iob_initialize();
#endif

  /* Register devices */

#if defined(CONFIG_DEV_NULL)
  devnull_register();   /* Standard /dev/null */
#endif

#if defined(CONFIG_DEV_ZERO)
  devzero_register();   /* Standard /dev/zero */
#endif

  /* Initialize the serial device driver */

#ifdef USE_SERIALDRIVER
  up_serialinit();
#endif

  /* Initialize the console device driver (if it is other than the standard
   * serial driver).
   */

#if defined(CONFIG_DEV_LOWCONSOLE)
  lowconsole_init();
#elif defined(CONFIG_CONSOLE_SYSLOG)
  syslog_console_init();
#elif defined(CONFIG_RAMLOG_CONSOLE)
  ramlog_consoleinit();
#endif

#ifdef CONFIG_PSEUDOTERM_SUSV1
  /* Register the master pseudo-terminal multiplexor device */

  (void)ptmx_register();
#endif

  /* Early initialization of the system logging device.  Some SYSLOG channel
   * can be initialized early in the initialization sequence because they
   * depend on only minimal OS initialization.
   */

  syslog_initialize(SYSLOG_INIT_EARLY);

#ifdef CONFIG_RAMLOG_SYSLOG
  ramlog_sysloginit();
#endif

  board_autoled_on(LED_IRQSENABLED);
}
