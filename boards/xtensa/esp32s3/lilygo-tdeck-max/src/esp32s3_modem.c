/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_modem.c
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
 * The modem's ring indicator (RI) and DTR, for light sleep with the modem
 * on.  The UART stops in light sleep; RI, low when the modem has something
 * to say (an incoming call or SMS; other URCs as AT+CFGRI sets), wakes the
 * chip and holds it awake while it stays low and for MODEM_RI_AWAKE_MS
 * after, so that what the modem says can come in.
 * DTR lets the modem sleep too (AT+CSCLK=1): high, it may; low, it stays
 * awake for commands.  While it sleeps, the modem keeps its URCs until DTR
 * goes low.
 *
 *   /dev/modem_ri   an interrupt pin (GPIOC_REGISTER, poll): low while the
 *                   modem signals
 *   /dev/modem_dtr  an output pin
 *
 * Both lines go through the modem's level shifter, which the modem's own
 * 1.8 V supplies: while the modem is off, RI reads low.  It is listened to
 * only while the modem's rail is on (tdeck_modem_listen()), and the low
 * level of a modem still starting keeps the chip awake meanwhile.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "lilygo-tdeck-max.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODEM_RI_AWAKE_MS 2000    /* Awake after RI goes low */
#define MODEM_RI_POLL_MS  30      /* Polled while RI stays low */

#ifdef CONFIG_SCHED_LPWORK
#  define MODEM_WORK      LPWORK
#else
#  define MODEM_WORK      HPWORK
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tdeck_dtr_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int  tdeck_dtr_write(FAR struct gpio_dev_s *dev, bool value);
static int  tdeck_ri_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int  tdeck_ri_attach(FAR struct gpio_dev_s *dev,
                            pin_interrupt_t callback);
static int  tdeck_ri_enable(FAR struct gpio_dev_s *dev, bool enable);
static int  tdeck_modem_setpintype(FAR struct gpio_dev_s *dev,
                                   enum gpio_pintype_e pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s g_dtr_ops =
{
  .go_read       = tdeck_dtr_read,
  .go_write      = tdeck_dtr_write,
  .go_setpintype = tdeck_modem_setpintype,
};

static const struct gpio_operations_s g_ri_ops =
{
  .go_read       = tdeck_ri_read,
  .go_attach     = tdeck_ri_attach,
  .go_enable     = tdeck_ri_enable,
  .go_setpintype = tdeck_modem_setpintype,
};

static struct gpio_dev_s g_dtr =
{
  .gp_pintype = GPIO_OUTPUT_PIN,
  .gp_ops     = &g_dtr_ops,
};

static struct gpio_dev_s g_ri =
{
  .gp_pintype = GPIO_INTERRUPT_LOW_PIN,
  .gp_ops     = &g_ri_ops,
};

static bool g_dtr_level;                /* The pad, an output, does not
                                         * read back */
static pin_interrupt_t g_ri_callback;   /* The upper half's, if attached */
static bool g_ri_enabled;               /* ... and enabled */
static bool g_listening;                /* The modem's rail is on */
static struct work_s g_ri_work;

PM_WAKELOCK_DECLARE_STATIC(g_ri_wakelock, "modem_ri", PM_IDLE_DOMAIN,
                           PM_IDLE);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_dtr_read / tdeck_dtr_write / tdeck_ri_read
 ****************************************************************************/

static int tdeck_dtr_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  *value = g_dtr_level;
  return OK;
}

static int tdeck_dtr_write(FAR struct gpio_dev_s *dev, bool value)
{
  g_dtr_level = value;
  esp_gpiowrite(BOARD_MODEM_DTR, value);
  return OK;
}

static int tdeck_ri_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  *value = esp_gpioread(BOARD_MODEM_RI);
  return OK;
}

/****************************************************************************
 * Name: tdeck_ri_attach / tdeck_ri_enable
 *
 * Description:
 *   The upper half's callback, for GPIOC_REGISTER.  The interrupt itself
 *   is on while the modem's rail is, whether anyone listens or not: it
 *   keeps the chip awake for what the modem says.
 *
 ****************************************************************************/

static int tdeck_ri_attach(FAR struct gpio_dev_s *dev,
                           pin_interrupt_t callback)
{
  irqstate_t flags = enter_critical_section();

  g_ri_callback = callback;
  leave_critical_section(flags);
  return OK;
}

static int tdeck_ri_enable(FAR struct gpio_dev_s *dev, bool enable)
{
  irqstate_t flags = enter_critical_section();

  g_ri_enabled = enable;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: tdeck_modem_setpintype
 ****************************************************************************/

static int tdeck_modem_setpintype(FAR struct gpio_dev_s *dev,
                                  enum gpio_pintype_e pintype)
{
  return pintype == dev->gp_pintype ? OK : -EINVAL;
}

/****************************************************************************
 * Name: tdeck_ri_worker
 *
 * Description:
 *   RI went low: wait for it to go back up, as the level interrupt would
 *   fire all the while, then listen again.
 *
 ****************************************************************************/

static void tdeck_ri_worker(FAR void *arg)
{
  irqstate_t flags;

  if (!esp_gpioread(BOARD_MODEM_RI))
    {
      /* Still low: awake for as long, and for a while after */

      if (g_listening)
        {
          pm_wakelock_staytimeout(&g_ri_wakelock, MODEM_RI_AWAKE_MS);
          work_queue(MODEM_WORK, &g_ri_work, tdeck_ri_worker, NULL,
                     MSEC2TICK(MODEM_RI_POLL_MS));
        }

      return;
    }

  flags = enter_critical_section();
  if (g_listening)
    {
      esp_gpioirqenable(BOARD_MODEM_RI);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tdeck_ri_interrupt
 ****************************************************************************/

static int tdeck_ri_interrupt(int irq, FAR void *context, FAR void *arg)
{
  esp_gpioirqdisable(BOARD_MODEM_RI);
  if (!g_listening)
    {
      return OK;
    }

  pm_wakelock_staytimeout(&g_ri_wakelock, MODEM_RI_AWAKE_MS);

  if (g_ri_callback != NULL && g_ri_enabled)
    {
      g_ri_callback(&g_ri, 0);
    }

  work_queue(MODEM_WORK, &g_ri_work, tdeck_ri_worker, NULL,
             MSEC2TICK(MODEM_RI_POLL_MS));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_modem_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

int tdeck_modem_initialize(void)
{
  int ret;

  /* DTR low: the modem stays awake for commands */

  g_dtr_level = false;
  esp_gpiowrite(BOARD_MODEM_DTR, false);
  esp_configgpio(BOARD_MODEM_DTR, OUTPUT);

  /* RI's interrupt attached, and off, before the pin is set to trigger:
   * RI reads low while the modem is off
   */

  esp_configgpio(BOARD_MODEM_RI, INPUT);
  ret = esp_gpio_irq(BOARD_MODEM_RI, tdeck_ri_interrupt, NULL);
  if (ret != OK)
    {
      return -EIO;
    }

  esp_gpioirqdisable(BOARD_MODEM_RI);
  esp_configgpio(BOARD_MODEM_RI, INPUT | ONLOW);
  esp_gpioirqdisable(BOARD_MODEM_RI);

  ret = gpio_pin_register_byname(&g_dtr, "modem_dtr");
  if (ret >= 0)
    {
      ret = gpio_pin_register_byname(&g_ri, "modem_ri");
    }

  return ret;
}

/****************************************************************************
 * Name: tdeck_modem_listen
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

void tdeck_modem_listen(bool on)
{
  irqstate_t flags = enter_critical_section();

  g_listening = on;
  if (on)
    {
      esp_gpioirqenable(BOARD_MODEM_RI);
    }
  else
    {
      esp_gpioirqdisable(BOARD_MODEM_RI);
    }

  leave_critical_section(flags);

  if (!on)
    {
      work_cancel(MODEM_WORK, &g_ri_work);

      /* DTR low, as at the start: a line driven high into the unpowered
       * level shifter would feed it
       */

      g_dtr_level = false;
      esp_gpiowrite(BOARD_MODEM_DTR, false);
    }

  tdeck_power_wake(BOARD_MODEM_RI, false, on);
}
