/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_lowput.c
 *
 *   Copyright 2019 NuttX. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "up_arch.h"
#include "up_internal.h"

//#include "rv32m1_clock.h"
//#include "rv32m1_uart.h"
//#include "rv32m1_gpio.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration **********************************************************/


/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowputhex32
 *
 * Description:
 *   Output 32bits in hex on the serial console
 *
 ****************************************************************************/

void up_lowputhex32(uint32_t hex32)
{
  int i;
  unsigned char j;

  up_lowputc('0');
  up_lowputc('x');
  for (i = 32 - 4; i >= 0; i -= 4)
  {
    j = (hex32 >> i) & 0xF;
    if (j > 9)
      j = j - 10 + 'A';
    else
      j += '0';

    up_lowputc(j);
  }

  up_lowputc(0x0D);
  up_lowputc(0x0A);
}

/****************************************************************************
 * Name: up_lowputhex16
 *
 * Description:
 *   Output 16bits in hex on the serial console
 *
 ****************************************************************************/

void up_lowputhex16(uint16_t hex16)
{
  int i;
  unsigned char j;

  up_lowputc('0');
  up_lowputc('x');
  for (i = 16 - 4; i >= 0; i -= 4)
  {
    j = (hex16 >> i) & 0xF;
    if (j > 9)
      j = j - 10 + 'A';
    else
      j += '0';

    up_lowputc(j);
  }

  up_lowputc(0x0D);
  up_lowputc(0x0A);
}

/****************************************************************************
 * Name: up_lowputhex8
 *
 * Description:
 *   Output 16bits in hex on the serial console
 *
 ****************************************************************************/

void up_lowputhex8(uint8_t hex8)
{
  int i;
  unsigned char j;

  up_lowputc('0');
  up_lowputc('x');
  for (i = 8 - 4; i >= 0; i -= 4)
  {
    j = (hex8 >> i) & 0xF;
    if (j > 9)
      j = j - 10 + 'A';
    else
      j += '0';

    up_lowputc(j);
  }

  up_lowputc(0x0D);
  up_lowputc(0x0A);
}

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
  rv32m1_lpuart0_putc(ch);
}

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output availabe as soon
 *   as possible.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
  rv32m1_ri5cy_board_init();
}



