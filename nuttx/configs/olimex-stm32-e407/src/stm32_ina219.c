/****************************************************************************
 * configs/olimex-stm32-e407/src/stm32_ina219.c
 *
 *   Copyright (C) 2018 Erle Robotics (Juan Flores Muñoz). All rights reserved.
 *   Author: Erle Robotics (Juan Flores Muñoz) <juan@erlerobotics.com>
 *
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

#include <errno.h>
#include <syslog.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/ina219.h>

#include "stm32.h"
#include "stm32_i2c.h"
#include "olimex-stm32-e407.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_INA219)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INA219_I2C_PORTNO 1   /* On I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ina219initialize
 *
 * Description:
 *   Initialize and register the INA219 voltage/current sensor.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/ina219"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_ina219initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing INA219!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(INA219_I2C_PORTNO);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the sensor */

  ret = ina219_register(devpath, i2c,0x40,100000,0x00);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Error registering hih6130\n");
    }

  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_INA219 */

