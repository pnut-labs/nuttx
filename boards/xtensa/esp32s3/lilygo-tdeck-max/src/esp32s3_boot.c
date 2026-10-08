/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_boot.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/param.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "lilygo-tdeck-max.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tdeck_pin_s
{
  uint8_t pin;
  bool    level;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lines set before any device is touched.  The e-paper, the microSD
 * card and the SX1262 share one SPI bus, so every chip select must idle
 * high before anything on the bus is powered or accessed; a chip select
 * held low on the unpowered SX1262 loads the bus, and the e-paper stops
 * answering.  The e-paper's reset is released; the SX1262's is held, since
 * its power rail (on the XL9555) is off at this point and a high reset line
 * would feed it.  The lights are off.
 */

static const struct tdeck_pin_s g_boot_pins[] =
{
  { BOARD_EPD_CS,             true  },
  { BOARD_SD_CS,              true  },
  { BOARD_LORA_CS,            true  },
  { BOARD_EPD_RST,            true  },
  { BOARD_EPD_DC,             true  },
  { BOARD_LORA_RST,           false },
  { BOARD_EPD_FRONTLIGHT,     false },
  { BOARD_KEYBOARD_BACKLIGHT, false },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_board_initialize
 *
 * Description:
 *   All ESP32-S3 boards must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void esp32s3_board_initialize(void)
{
  size_t i;

  for (i = 0; i < nitems(g_boot_pins); i++)
    {
      /* Set the level before the output is enabled, so that a chip
       * select never glitches low.
       */

      esp_gpiowrite(g_boot_pins[i].pin, g_boot_pins[i].level);
      esp_configgpio(g_boot_pins[i].pin, OUTPUT);
    }
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  esp32s3_bringup();
}
#endif
