/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_cst3530.c
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

/* The touch screen: a CST3530 on the I2C bus, over the e-paper panel, with
 * three keys on the glass below it.  Its interrupt is on GPIO 12, and its
 * reset on one of the XL9555's lines.  The vendor's firmware maps the
 * touches onto the panel's pixels as they are: no swapping or mirroring.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/cst3530.h>
#include <nuttx/input/kbd_codec.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "lilygo-tdeck-max.h"

#if defined(CONFIG_INPUT_CST3530) && defined(CONFIG_IOEXPANDER_PCA9555)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CST3530_FREQUENCY   400000

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tdeck_touch_attach(FAR const struct cst3530_config_s *config,
                               xcpt_t isr, FAR void *arg);
static void tdeck_touch_enable(FAR const struct cst3530_config_s *config,
                               bool enable);
static int  tdeck_touch_reset(FAR const struct cst3530_config_s *config,
                              bool assert);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The glass keys are the device's soft keys, numbered from the left: the
 * screen shows what each does now.  Entry n is the key with id n.
 */

static const uint32_t g_softkeys[] =
{
  KEYCODE_F1, KEYCODE_F2, KEYCODE_F3
};

static const struct cst3530_config_s g_touch =
{
  .frequency   = CST3530_FREQUENCY,
  .address     = CST3530_ADDRESS,
  .flags       = 0,
  .tkeys_path  = "/dev/softkeys",
  .tkeys_codes = g_softkeys,
  .ntkeys      = sizeof(g_softkeys) / sizeof(g_softkeys[0]),
  .attach      = tdeck_touch_attach,
  .enable      = tdeck_touch_enable,
  .reset       = tdeck_touch_reset,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_touch_attach
 *
 * Description:
 *   Attach the handler, and leave the interrupt disabled until the driver
 *   enables it: esp_gpio_irq() enables it as it attaches.
 *
 ****************************************************************************/

static int tdeck_touch_attach(FAR const struct cst3530_config_s *config,
                              xcpt_t isr, FAR void *arg)
{
  int ret;

  /* It returns 1 (-ERROR), not a negated errno, when it fails */

  ret = esp_gpio_irq(BOARD_TOUCH_INT, isr, arg);
  if (ret != OK)
    {
      return ret < 0 ? ret : -EIO;
    }

  esp_gpioirqdisable(BOARD_TOUCH_INT);
  return OK;
}

/****************************************************************************
 * Name: tdeck_touch_enable
 ****************************************************************************/

static void tdeck_touch_enable(FAR const struct cst3530_config_s *config,
                               bool enable)
{
  if (enable)
    {
      esp_gpioirqenable(BOARD_TOUCH_INT);
    }
  else
    {
      esp_gpioirqdisable(BOARD_TOUCH_INT);
    }
}

/****************************************************************************
 * Name: tdeck_touch_reset
 ****************************************************************************/

static int tdeck_touch_reset(FAR const struct cst3530_config_s *config,
                             bool assert)
{
  /* Low holds the controller in reset */

  return tdeck_xl9555_write(XL9555_TOUCH_RST, !assert);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_touch_initialize
 *
 * Description:
 *   Register the touch screen as /dev/input0 and the glass keys as
 *   /dev/softkeys.  The interrupt is taken on the low level, as the
 *   keyboard's is: the driver masks it until it has read the report.
 *
 ****************************************************************************/

int tdeck_touch_initialize(FAR struct i2c_master_s *i2c)
{
  esp_configgpio(BOARD_TOUCH_INT, INPUT | PULLUP | ONLOW);

  return cst3530_register(i2c, &g_touch, "/dev/input0");
}

#endif /* CONFIG_INPUT_CST3530 && CONFIG_IOEXPANDER_PCA9555 */
