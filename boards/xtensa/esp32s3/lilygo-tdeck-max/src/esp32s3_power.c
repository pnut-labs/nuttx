/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_power.c
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
 * Power management, on NuttX's framework.  The chip light-sleeps
 * (PM_STANDBY) whenever nothing holds it awake, until its next timer or a
 * wake-up pin.  Every state is held from the start
 * (PM_GOVERNOR_EXPLICIT_RELAX=-1); once the board is up, PM_NORMAL and
 * PM_IDLE are released.  PM_STANDBY stays held for good: PM_SLEEP is a
 * deep sleep, which on this board would amount to a restart.
 *
 * What keeps the chip awake, each a wakelock in /proc/pm/wakelock0:
 *
 *   usb                  USB power (esp32s3_battery.c)
 *   gps_en, modem_pwr    the GNSS receiver's and the modem's rails
 *                        (esp32s3_xl9555.c)
 *   audio_out, audio_in  an audio device reserved (esp32s3_audio.c)
 *   wifi                 Wi-Fi up (esp32s3_radios.c); BLE holds the chip
 *                        awake itself
 *   lights               the front light or the keyboard's backlight on
 *   awake                /dev/awake open
 *
 * Console input also keeps the chip awake for a few seconds
 * (pm_activity()).
 *
 * A pin wakes the chip while its driver has the pin's interrupt enabled:
 * the keyboard's, the touch screen's while it is open, the LoRa radio's
 * DIO1 and the IMU's.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_ESPRESSIF_LEDC
#  include <nuttx/timers/pwm.h>
#  include "espressif/esp_ledc.h"
#endif

#include "espressif/esp_pm.h"
#include "lilygo-tdeck-max.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tdeck_awake_open(FAR struct file *filep);
static int tdeck_awake_close(FAR struct file *filep);
static ssize_t tdeck_awake_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);

#ifdef CONFIG_ESPRESSIF_LEDC
static int tdeck_lights_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int tdeck_lights_start(FAR struct pwm_lowerhalf_s *dev,
                              FAR const struct pwm_info_s *info);
static int tdeck_lights_stop(FAR struct pwm_lowerhalf_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The GPIOs that wake the chip */

static uint64_t g_wake_pins;

/* /dev/awake: the chip stays awake while it is open (read-only) */

static const struct file_operations g_awake_fops =
{
  tdeck_awake_open,             /* open */
  tdeck_awake_close,            /* close */
  tdeck_awake_read,             /* read */
};

PM_WAKELOCK_DECLARE_STATIC(g_awake_wakelock, "awake", PM_IDLE_DOMAIN,
                           PM_IDLE);

#ifdef CONFIG_ESPRESSIF_LEDC
/* The lights' PWM operations, with start, stop and shutdown extended */

static struct pwm_ops_s g_lights_ops;
static FAR const struct pwm_ops_s *g_lights_orig;
PM_WAKELOCK_DECLARE_STATIC(g_lights_wakelock, "lights", PM_IDLE_DOMAIN,
                           PM_IDLE);
static bool g_lights_on;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_awake_open / tdeck_awake_close
 *
 * Description:
 *   Each open of /dev/awake holds the chip out of light sleep until it is
 *   closed, which also happens when the task that opened it ends.
 *
 ****************************************************************************/

static int tdeck_awake_open(FAR struct file *filep)
{
  pm_wakelock_stay(&g_awake_wakelock);
  return OK;
}

static int tdeck_awake_close(FAR struct file *filep)
{
  pm_wakelock_relax(&g_awake_wakelock);
  return OK;
}

/****************************************************************************
 * Name: tdeck_awake_read
 *
 * Description:
 *   There is nothing to read, but a device without a read method cannot be
 *   opened read-only.
 *
 ****************************************************************************/

static ssize_t tdeck_awake_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  return 0;
}

#ifdef CONFIG_ESPRESSIF_LEDC
/****************************************************************************
 * Name: tdeck_lights_set
 *
 * Description:
 *   The LED controller stops in light sleep, and the lights with it: keep
 *   the chip awake while one of them is on.
 *
 ****************************************************************************/

static void tdeck_lights_set(bool on)
{
  irqstate_t flags = enter_critical_section();

  if (on != g_lights_on)
    {
      if (on)
        {
          pm_wakelock_stay(&g_lights_wakelock);
        }
      else
        {
          pm_wakelock_relax(&g_lights_wakelock);
        }

      g_lights_on = on;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tdeck_lights_shutdown / tdeck_lights_start / tdeck_lights_stop
 ****************************************************************************/

static int tdeck_lights_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  int ret = g_lights_orig->shutdown(dev);

  tdeck_lights_set(false);
  return ret;
}

static int tdeck_lights_start(FAR struct pwm_lowerhalf_s *dev,
                              FAR const struct pwm_info_s *info)
{
  bool on = false;
  int ret;
  int i;

  ret = g_lights_orig->start(dev, info);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      if (info->channels[i].duty > 0)
        {
          on = true;
        }
    }

  tdeck_lights_set(on);
  return ret;
}

static int tdeck_lights_stop(FAR struct pwm_lowerhalf_s *dev)
{
  int ret = g_lights_orig->stop(dev);

  tdeck_lights_set(false);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_power_wake
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

void tdeck_power_wake(int pin, bool high, bool enable)
{
  uint64_t bit = UINT64_C(1) << pin;
  irqstate_t flags;
  int ret = OK;

  /* A driver masks its interrupt in its handler until its worker has
   * dealt with the part: the pin keeps waking the chip meanwhile.
   */

  if (up_interrupt_context())
    {
      return;
    }

  flags = enter_critical_section();

  if (((g_wake_pins & bit) != 0) != enable)
    {
      ret = esp_pm_wakeup_gpio(pin, high, enable);
      if (ret >= 0)
        {
          g_wake_pins ^= bit;
        }
    }

  leave_critical_section(flags);

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: GPIO%d wake-up %s: %d\n", pin,
             enable ? "on" : "off", ret);
    }
}

#ifdef CONFIG_ESPRESSIF_LEDC
/****************************************************************************
 * Name: tdeck_lights_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

int tdeck_lights_initialize(void)
{
  FAR struct pwm_lowerhalf_s *pwm;

  pwm = esp_ledc_init(0);
  if (pwm == NULL)
    {
      return -ENODEV;
    }

  g_lights_orig         = pwm->ops;
  g_lights_ops          = *pwm->ops;
  g_lights_ops.shutdown = tdeck_lights_shutdown;
  g_lights_ops.start    = tdeck_lights_start;
  g_lights_ops.stop     = tdeck_lights_stop;
  pwm->ops              = &g_lights_ops;

  return pwm_register("/dev/pwm0", pwm);
}
#endif

/****************************************************************************
 * Name: tdeck_power_initialize
 *
 * Description:
 *   See src/lilygo-tdeck-max.h
 *
 ****************************************************************************/

void tdeck_power_initialize(void)
{
  int ret;

  ret = register_driver("/dev/awake", &g_awake_fops, 0444, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/awake: %d\n", ret);
    }

#if CONFIG_PM_GOVERNOR_EXPLICIT_RELAX < 0
  /* Every state has been held since the start: allow light sleep */

  pm_relax(PM_IDLE_DOMAIN, PM_NORMAL);
  pm_relax(PM_IDLE_DOMAIN, PM_IDLE);
#else
  /* Never deep sleep */

  pm_stay(PM_IDLE_DOMAIN, PM_STANDBY);
#endif
}

#endif /* CONFIG_PM */
