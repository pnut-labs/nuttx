/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_imu.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * The BHI260AP IMU is on the shared I2C bus at 0x28, powered by the
 * XL9555's 1V8_EN line (on from boot), with its host interrupt HIRQ on
 * GPIO21; its reset is not wired.  The driver uploads Bosch's firmware image
 * to it the first time a sensor is activated.  The image is not kept here:
 * the build downloads it (see Make.defs), and esp32s3_imu_fw.S includes it.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bhi260ap.h>

#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "esp32s3_i2c.h"

#include "lilygo-tdeck-max.h"

#ifdef CONFIG_SENSORS_BHI260AP

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tdeck_imu_attach(FAR const struct bhi260ap_config_s *config,
                             xcpt_t isr, FAR void *arg);
static void tdeck_imu_enable(FAR const struct bhi260ap_config_s *config,
                             bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The firmware image, from esp32s3_imu_fw.S */

extern const uint8_t g_bhi260ap_firmware[];
extern const uint8_t g_bhi260ap_firmware_end[];

/* The image's address and size are filled in when the driver is
 * registered: the size is not a constant the compiler knows
 */

static struct bhi260ap_config_s g_bhi260ap_config =
{
  .frequency     = 400000,
  .attach        = tdeck_imu_attach,
  .enable        = tdeck_imu_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_imu_attach
 ****************************************************************************/

static int tdeck_imu_attach(FAR const struct bhi260ap_config_s *config,
                            xcpt_t isr, FAR void *arg)
{
  int ret;

  ret = esp_gpio_irq(BOARD_IMU_INT, isr, arg);
  if (ret < 0)
    {
      snerr("ERROR: Failed to attach GPIO%d: %d\n", BOARD_IMU_INT, ret);
      return ret;
    }

  esp_gpioirqdisable(BOARD_IMU_INT);
  return OK;
}

/****************************************************************************
 * Name: tdeck_imu_enable
 ****************************************************************************/

static void tdeck_imu_enable(FAR const struct bhi260ap_config_s *config,
                             bool enable)
{
  if (enable)
    {
      esp_gpioirqenable(BOARD_IMU_INT);
    }
  else
    {
      esp_gpioirqdisable(BOARD_IMU_INT);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_imu_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

int tdeck_imu_initialize(void)
{
  FAR struct i2c_master_s *i2c;

  /* HIRQ is high while the chip has data for the host, until the driver
   * has read it: level triggered.  It stays low while no firmware runs.
   */

  esp_configgpio(BOARD_IMU_INT, INPUT | PULLDOWN | ONHIGH);

  i2c = esp32s3_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  g_bhi260ap_config.firmware      = g_bhi260ap_firmware;
  g_bhi260ap_config.firmware_size = g_bhi260ap_firmware_end -
                                    g_bhi260ap_firmware;

  return bhi260ap_register(0, i2c, BHI260AP_I2C_ADDRESS,
                           &g_bhi260ap_config);
}

#endif /* CONFIG_SENSORS_BHI260AP */
