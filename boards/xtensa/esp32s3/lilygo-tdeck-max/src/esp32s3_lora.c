/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_lora.c
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
 * The SX1262 LoRa radio shares SPI2 with the e-paper and the microSD card:
 * NSS is GPIO3, NRESET GPIO4, DIO1 (the interrupt) GPIO5 and BUSY GPIO6.
 * The module's 32 MHz TCXO is powered from DIO3 at 2.4 V and DIO2 drives
 * its RF switch, as in the vendor's examples.  This is the 868 MHz
 * variant; the board only allows 863-870 MHz.
 *
 * The rail (lora_en) is on from boot with the radio asleep
 * (tdeck_lora_sleep()); the driver resets it when /dev/lora0 is opened and
 * puts it back to sleep when it is closed.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wireless/lpwan/sx126x.h>

#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "esp32s3_spi.h"

#include "lilygo-tdeck-max.h"

#ifdef CONFIG_LPWAN_SX126X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The band this module is built for, and the chip's power range (dBm) */

#define LORA_FREQ_MIN        863000000
#define LORA_FREQ_MAX        870000000
#define LORA_POWER_MIN       (-9)
#define LORA_POWER_MAX       22

/* The TCXO needs up to 5 ms to start, in steps of 15.625 us */

#define LORA_TCXO_DELAY      320

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tdeck_lora_reset(void);
static bool tdeck_lora_busy(void);
static int  tdeck_lora_irq0attach(xcpt_t handler, FAR void *arg);
static void tdeck_lora_irq0enable(bool enable);
static int  tdeck_lora_get_pa_values(FAR enum sx126x_device_e *model,
                                        FAR uint8_t *hpmax,
                                        FAR uint8_t *padutycycle);
static int  tdeck_lora_limit_tx_power(FAR uint8_t *power);
static int  tdeck_lora_check_frequency(uint32_t frequency);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sx126x_lower_s g_lora_lower =
{
  .dev_number       = 0,
  .reset            = tdeck_lora_reset,
  .busy             = tdeck_lora_busy,

  /* Every IRQ the driver enables goes to DIO1 */

  .masks =
    {
      .dio1_mask    = 0xffff,
      .dio2_mask    = 0,
      .dio3_mask    = 0,
    },

  .dio3_voltage     = SX126X_TCXO_2_4V,
  .dio3_delay       = LORA_TCXO_DELAY,
  .use_dio2_as_rf_sw = true,
  .irq0attach       = tdeck_lora_irq0attach,
  .irq0enable       = tdeck_lora_irq0enable,
  .regulator_mode   = SX126X_DC_DC_LDO,
  .get_pa_values    = tdeck_lora_get_pa_values,
  .limit_tx_power   = tdeck_lora_limit_tx_power,
  .tx_ramp_time     = SX126X_SET_RAMP_200U,
  .check_frequency  = tdeck_lora_check_frequency,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_lora_reset
 ****************************************************************************/

static void tdeck_lora_reset(void)
{
  /* At least 100 us low; the driver then waits for BUSY to drop */

  esp_gpiowrite(BOARD_LORA_RST, false);
  up_udelay(200);
  esp_gpiowrite(BOARD_LORA_RST, true);
  up_udelay(200);
}

/****************************************************************************
 * Name: tdeck_lora_busy
 ****************************************************************************/

static bool tdeck_lora_busy(void)
{
  return esp_gpioread(BOARD_LORA_BUSY);
}

/****************************************************************************
 * Name: tdeck_lora_irq0attach
 ****************************************************************************/

static int tdeck_lora_irq0attach(xcpt_t handler, FAR void *arg)
{
  int ret;

  ret = esp_gpio_irq(BOARD_LORA_DIO1, handler, arg);
  if (ret < 0)
    {
      wlerr("ERROR: Failed to attach GPIO%d: %d\n", BOARD_LORA_DIO1, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: tdeck_lora_irq0enable
 ****************************************************************************/

static void tdeck_lora_irq0enable(bool enable)
{
  if (enable)
    {
      esp_gpioirqenable(BOARD_LORA_DIO1);
    }
  else
    {
      esp_gpioirqdisable(BOARD_LORA_DIO1);
    }
}

/****************************************************************************
 * Name: tdeck_lora_get_pa_values
 ****************************************************************************/

static int tdeck_lora_get_pa_values(FAR enum sx126x_device_e *model,
                                       FAR uint8_t *hpmax,
                                       FAR uint8_t *padutycycle)
{
  /* The datasheet's setting for up to +22 dBm (table 13-21); lower powers
   * are set with SetTxParams.
   */

  *model       = SX1262;
  *hpmax       = 0x07;
  *padutycycle = 0x04;
  return OK;
}

/****************************************************************************
 * Name: tdeck_lora_limit_tx_power
 ****************************************************************************/

static int tdeck_lora_limit_tx_power(FAR uint8_t *power)
{
  int8_t dbm = (int8_t)*power;

  if (dbm < LORA_POWER_MIN)
    {
      dbm = LORA_POWER_MIN;
    }
  else if (dbm > LORA_POWER_MAX)
    {
      dbm = LORA_POWER_MAX;
    }

  *power = (uint8_t)dbm;
  return OK;
}

/****************************************************************************
 * Name: tdeck_lora_check_frequency
 ****************************************************************************/

static int tdeck_lora_check_frequency(uint32_t frequency)
{
  return frequency >= LORA_FREQ_MIN && frequency <= LORA_FREQ_MAX ?
         OK : -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_lora_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

int tdeck_lora_initialize(void)
{
  FAR struct spi_dev_s *spi;

  /* DIO1 is high while the chip has an IRQ pending, until the driver
   * clears it: level triggered.  The pull-down keeps it quiet while the
   * rail is off.
   */

  esp_configgpio(BOARD_LORA_DIO1, INPUT | PULLDOWN | ONHIGH);
  esp_configgpio(BOARD_LORA_BUSY, INPUT);

  spi = esp32s3_spibus_initialize(ESP32S3_SPI2);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  sx126x_register(spi, &g_lora_lower, "/dev/lora0");
  return OK;
}

#endif /* CONFIG_LPWAN_SX126X */
