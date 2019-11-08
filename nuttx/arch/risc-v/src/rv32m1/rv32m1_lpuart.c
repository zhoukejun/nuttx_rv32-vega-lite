/****************************************************************************
 * arch/risc-v/src/rv32m1/rv32m1_lpuart.c
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

#include "fsl_clock.h"
#include "fsl_lpuart.h"

#if 0
/* Array of LPUART clock name. */
/*! @brief Clock ip name array for LPUART. */
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


static uint32_t *const s_lpuartBases[] = LPUART_BASE_PTRS;
static const clock_ip_name_t s_lpuartClock[] = LPUART_CLOCKS;

uint32_t LPUART_GetInstance(LPUART_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_lpuartBases); instance++)
    {
        if (s_lpuartBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_lpuartBases));

    return instance;
}

void rv32m1_lpuart_get_default_config(lpuart_config_t *config)
{
  assert(config);

  config->baudRate_Bps = 115200U;
  config->parityMode = LPUART_PARITY_DISABLED;
  config->dataBitsCount = LPUART_EIGHT_DATA_BITS;
  config->isMsb = false;
  config->stopBitCount = LPUART_ONE_STOP_BIT;
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
  config->txFifoWatermark = 0; 
  config->rxFifoWatermark = 0; 
#endif
  config->rxIdleType = LPUART_IDLE_TYPE_START_BIT;
  config->rxIdleConfig = LPUART_IDLE_CHARACTER1;
  config->enableTx = false;
  config->enableRx = false;
}

int rv32m1_lpuart_set(LPUART_Type *base, lpuart_config_t *config)
{
  assert(config);
  assert(config->baudRate_Bps);
#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
  assert(FSL_FEATURE_LPUART_FIFO_SIZEn(base) >= config->txFifoWatermark);
  assert(FSL_FEATURE_LPUART_FIFO_SIZEn(base) >= config->rxFifoWatermark);
#endif

  uint32_t temp;
  uint16_t sbr, sbrTemp;
  uint32_t osr, osrTemp, tempDiff, calculatedBaud, baudDiff;

  uint32_t srcClock_Hz = CLOCK_GetIpFreq(CLOCK_LPUART0);

  /* This LPUART instantiation uses a slightly different baud rate calculation
   * The idea is to use the best OSR (over-sampling rate) possible
   * Note, OSR is typically hard-set to 16 in other LPUART instantiations
   * loop to find the best OSR value possible, one that generates minimum baudDiff
   * iterate through the rest of the supported values of OSR */

  baudDiff = config->baudRate_Bps;
  osr = 0;
  sbr = 0;
  for (osrTemp = 4; osrTemp <= 32; osrTemp++)
  {
    /* calculate the temporary sbr value   */
    sbrTemp = (srcClock_Hz / (config->baudRate_Bps * osrTemp));
    /*set sbrTemp to 1 if the sourceClockInHz can not satisfy the desired baud rate*/
    if (sbrTemp == 0)
    {
      sbrTemp = 1;
    }
    /* Calculate the baud rate based on the temporary OSR and SBR values */
    calculatedBaud = (srcClock_Hz / (osrTemp * sbrTemp));

    tempDiff = calculatedBaud - config->baudRate_Bps;

    /* Select the better value between srb and (sbr + 1) */
    if (tempDiff > (config->baudRate_Bps - (srcClock_Hz / (osrTemp * (sbrTemp + 1)))))
    {
      tempDiff = config->baudRate_Bps - (srcClock_Hz / (osrTemp * (sbrTemp + 1)));
      sbrTemp++;
    }

    if (tempDiff <= baudDiff)
    {
      baudDiff = tempDiff;
      osr = osrTemp; /* update and store the best OSR value calculated */
      sbr = sbrTemp; /* update store the best SBR value calculated */
    }
  }

  /* Check to see if actual baud rate is within 3% of desired baud rate
   * based on the best calculate OSR value */
  if (baudDiff > ((config->baudRate_Bps / 100) * 3))
  {
    /* Unacceptable baud rate difference of more than 3%*/
    return ERROR;
  }

  uint32_t instance = LPUART_GetInstance(base);

  /* Enable lpuart clock */
  CLOCK_EnableClock(s_lpuartClock[instance]);

#if defined(FSL_FEATURE_LPUART_HAS_GLOBAL) && FSL_FEATURE_LPUART_HAS_GLOBAL
  /*Reset all internal logic and registers, except the Global Register */
  LPUART_SoftwareReset(base);
#else
  /* Disable LPUART TX RX before setting. */
  base->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
#endif

  temp = base->BAUD;

  /* Acceptable baud rate, check if OSR is between 4x and 7x oversampling.
   * If so, then "BOTHEDGE" sampling must be turned on */
  if ((osr > 3) && (osr < 8))
  {
    temp |= LPUART_BAUD_BOTHEDGE_MASK;
  }

  /* program the osr value (bit value is one less than actual value) */
  temp &= ~LPUART_BAUD_OSR_MASK;
  temp |= LPUART_BAUD_OSR(osr - 1);

  /* write the sbr value to the BAUD registers */
  temp &= ~LPUART_BAUD_SBR_MASK;
  base->BAUD = temp | LPUART_BAUD_SBR(sbr);

  /* Set bit count and parity mode. */
  base->BAUD &= ~LPUART_BAUD_M10_MASK;

  temp = base->CTRL &
         ~(LPUART_CTRL_PE_MASK | LPUART_CTRL_PT_MASK | LPUART_CTRL_M_MASK | LPUART_CTRL_ILT_MASK |
           LPUART_CTRL_IDLECFG_MASK);

  temp |=
      (uint8_t)config->parityMode | LPUART_CTRL_IDLECFG(config->rxIdleConfig) | LPUART_CTRL_ILT(config->rxIdleType);

  if (LPUART_PARITY_DISABLED != config->parityMode)
  {
    temp |= LPUART_CTRL_M_MASK; /* Eight data bits and one parity bit */
  }

  base->CTRL = temp;

  /* set stop bit per char */
  temp = base->BAUD & ~LPUART_BAUD_SBNS_MASK;
  base->BAUD = temp | LPUART_BAUD_SBNS((uint8_t)config->stopBitCount);

#if defined(FSL_FEATURE_LPUART_HAS_FIFO) && FSL_FEATURE_LPUART_HAS_FIFO
    /* Set tx/rx WATER watermark
       Note:
       Take care of the RX FIFO, RX interrupt request only assert when received bytes
       equal or more than RX water mark, there is potential issue if RX water
       mark larger than 1.
       For example, if RX FIFO water mark is 2, upper layer needs 5 bytes and
       5 bytes are received. the last byte will be saved in FIFO but not trigger
       RX interrupt because the water mark is 2.
     */
    base->WATER = (((uint32_t)(config->rxFifoWatermark) << 16) | config->txFifoWatermark);

    /* Enable tx/rx FIFO */
    base->FIFO |= (LPUART_FIFO_TXFE_MASK | LPUART_FIFO_RXFE_MASK);

    /* Flush FIFO */
    base->FIFO |= (LPUART_FIFO_TXFLUSH_MASK | LPUART_FIFO_RXFLUSH_MASK);
#endif

  /* Clear all status flags */
  temp = (LPUART_STAT_RXEDGIF_MASK | LPUART_STAT_IDLE_MASK | LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK |
    LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK);

  /* Set data bits order. */
  if (config->isMsb)
  {
    temp |= LPUART_STAT_MSBF_MASK;
  }
  else
  {
    temp &= ~LPUART_STAT_MSBF_MASK;
  }

  base->STAT |= temp;

  /* Enable TX/RX base on configure structure. */
  temp = base->CTRL;
  if (config->enableTx)
  {
    temp |= LPUART_CTRL_TE_MASK;
  }

  if (config->enableRx)
  {
    temp |= LPUART_CTRL_RE_MASK;
  }

  base->CTRL = temp;

  return OK;
}


void rv32m1_lpuart0_config(void)
{
  lpuart_config_t lpuart_config;

//  rv32m1_lpuart0_pin_config();

  rv32m1_lpuart_get_default_config(&lpuart_config);

  rv32m1_lpuart_set(LPUART0, &lpuart_config);
}

#endif

void rv32m1_lpuart0_putc(char ch)
{
#if 0
  while(!(getreg32(LPUART0 + LPUART_STAT_OFFSIZE) & LPUART_STAT_TDRE_MASK))
  {
  }
  putreg32(ch, LPUART0 + LPUART_DATA_OFFSIZE);
#endif

  LPUART_Type *base;
  base = (LPUART_Type *)LPUART0_BASE;
  while (!(base->STAT & LPUART_STAT_TDRE_MASK))
  {
  }
  base->DATA = ch;
}

