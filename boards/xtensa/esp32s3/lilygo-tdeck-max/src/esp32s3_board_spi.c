/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_board_spi.c
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
#include <stdint.h>

#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "esp32s3_spi.h"
#include "lilygo-tdeck-max.h"

#ifdef CONFIG_ESP32S3_SPI2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SX1262's SetSleep with a cold start: everything but the wake-up
 * logic off, about 160 nA.
 */

#define SX1262_SETSLEEP       0x84
#define SX1262_SLEEP_COLD     0x00

#define SX1262_BUSY_TIMEOUT   100     /* ms, after power-up */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_spi2_cs
 *
 * Description:
 *   The chip select of a device on SPI2, or -1.
 *
 ****************************************************************************/

static int tdeck_spi2_cs(uint32_t devid)
{
  switch (devid)
    {
      case SPIDEV_DISPLAY(0):
        return BOARD_EPD_CS;

      case SPIDEV_MMCSD(0):
        return BOARD_SD_CS;

      case SPIDEV_LPWAN(0):
        return BOARD_LORA_CS;

      default:
        return -1;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_spi2_select
 *
 * Description:
 *   Drive the chip select of one of the three devices on SPI2: the
 *   e-paper, the microSD card and the SX1262 (active low).  For any other
 *   device, as the driver asks while it starts, every one is released.
 *
 ****************************************************************************/

void esp32s3_spi2_select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  int cs = tdeck_spi2_cs(devid);

  if (cs >= 0)
    {
      esp_gpiowrite(cs, !selected);
    }
  else if (!selected)
    {
      esp_gpiowrite(BOARD_EPD_CS, true);
      esp_gpiowrite(BOARD_SD_CS, true);
      esp_gpiowrite(BOARD_LORA_CS, true);
    }
}

/****************************************************************************
 * Name: esp32s3_spi2_status
 *
 * Description:
 *   The microSD slot has no card-detect switch: report a card, and let the
 *   card driver find out.
 *
 ****************************************************************************/

uint8_t esp32s3_spi2_status(struct spi_dev_s *dev, uint32_t devid)
{
  return devid == SPIDEV_MMCSD(0) ? SPI_STATUS_PRESENT : 0;
}

/****************************************************************************
 * Name: esp32s3_spi2_cmddata
 *
 * Description:
 *   The e-paper's data/command line: low for a command.  spi_transfer(),
 *   behind /dev/spiN, calls this for every device and gives up if it
 *   fails, so a data phase succeeds for the devices without such a line.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int esp32s3_spi2_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      esp_gpiowrite(BOARD_EPD_DC, !cmd);
      return OK;
    }

  return cmd ? -ENODEV : OK;
}
#endif

/****************************************************************************
 * Name: tdeck_lora_sleep
 *
 * Description:
 *   Put the SX1262 to sleep, once its rail is on: it otherwise idles in
 *   standby.  It takes commands once its BUSY line is low, a few
 *   milliseconds after power-up, and holds BUSY high while it sleeps.
 *
 * Returned Value:
 *   Zero (OK), -ETIMEDOUT if it never got ready, or -EIO if it did not go
 *   to sleep.
 *
 ****************************************************************************/

int tdeck_lora_sleep(void)
{
  static const uint8_t setsleep[] =
  {
    SX1262_SETSLEEP, SX1262_SLEEP_COLD
  };

  FAR struct spi_dev_s *spi;
  int waited;

  spi = esp32s3_spibus_initialize(ESP32S3_SPI2);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  esp_configgpio(BOARD_LORA_BUSY, INPUT);

  for (waited = 0; esp_gpioread(BOARD_LORA_BUSY); waited++)
    {
      if (waited >= SX1262_BUSY_TIMEOUT)
        {
          return -ETIMEDOUT;
        }

      nxsig_usleep(1000);
    }

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, 1000000);
  SPI_SELECT(spi, SPIDEV_LPWAN(0), true);
  SPI_SNDBLOCK(spi, setsleep, sizeof(setsleep));
  SPI_SELECT(spi, SPIDEV_LPWAN(0), false);
  SPI_LOCK(spi, false);

  nxsig_usleep(1000);
  return esp_gpioread(BOARD_LORA_BUSY) ? OK : -EIO;
}

#endif /* CONFIG_ESP32S3_SPI2 */
