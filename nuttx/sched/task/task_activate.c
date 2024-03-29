/****************************************************************************
 * sched/task/task_activate.c
 *
 *   Copyright (C) 2007-2009, 2016 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_activate
 *
 * Description:
 *   This function activates tasks initialized by nxtask_schedsetup(). Without
 *   activation, a task is ineligible for execution by the scheduler.
 *
 * Input Parameters:
 *   tcb - The TCB for the task for the task (same as the task_init argument).
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

int task_activate(FAR struct tcb_s *tcb)
{
  irqstate_t flags = enter_critical_section();

up_lowputc('9');
up_lowputc('8');
up_lowputc('7');
#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Check if this is really a re-start */

  if (tcb->task_state != TSTATE_TASK_INACTIVE)
    {
      /* Inform the instrumentation layer that the task
       * has stopped
       */

      sched_note_stop(tcb);
    }

  /* Inform the instrumentation layer that the task
   * has started
   */

  sched_note_start(tcb);
#endif

  up_unblock_task(tcb);
  leave_critical_section(flags);
up_lowputc('7');
up_lowputc('8');
up_lowputc('9');
  return OK;
}
