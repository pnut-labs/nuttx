/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_board_lcd.c
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

#include <errno.h>
#include <stdbool.h>

#include <nuttx/board.h>
#include <nuttx/debug.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/uc8253.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "esp32s3_spi.h"
#include "lilygo-tdeck-max.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tdeck_epd_set_rst(bool on);
static bool tdeck_epd_check_busy(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct lcd_dev_s *g_lcddev;

static const struct uc8253_priv_s g_epd =
{
  .set_rst    = tdeck_epd_set_rst,
  .check_busy = tdeck_epd_check_busy,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_epd_set_rst
 *
 * Description:
 *   The panel's reset line, active low: on asserts reset.
 *
 ****************************************************************************/

static void tdeck_epd_set_rst(bool on)
{
  esp_gpiowrite(BOARD_EPD_RST, !on);
}

/****************************************************************************
 * Name: tdeck_epd_check_busy
 *
 * Description:
 *   The panel holds BUSY low while it works.
 *
 ****************************************************************************/

static bool tdeck_epd_check_busy(void)
{
  return !esp_gpioread(BOARD_EPD_BUSY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Bind the e-paper panel to SPI2.  Its reset and data/command lines were
 *   set by the early boot code; BUSY is set up here.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  FAR struct spi_dev_s *spi;

  if (g_lcddev != NULL)
    {
      return OK;
    }

  esp_configgpio(BOARD_EPD_BUSY, INPUT | PULLUP);

  spi = esp32s3_spibus_initialize(ESP32S3_SPI2);
  if (spi == NULL)
    {
      lcderr("ERROR: Failed to initialize SPI2\n");
      return -ENODEV;
    }

  g_lcddev = uc8253_initialize(spi, &g_epd);
  if (g_lcddev == NULL)
    {
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int devno)
{
  return devno == 0 ? g_lcddev : NULL;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 *
 * Description:
 *   Power the panel down; it keeps its image.
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  if (g_lcddev != NULL)
    {
      g_lcddev->setpower(g_lcddev, 0);
    }
}
