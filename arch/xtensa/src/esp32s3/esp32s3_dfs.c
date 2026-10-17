/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_dfs.c
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
 * Dynamic frequency scaling, built on the HAL's power management.  The HAL
 * keeps the CPU at its full frequency while a lock is held.  The idle loop
 * (up_idle()) releases it before waiting for an interrupt, and every
 * interrupt takes it again (xtensa_int_decode()), so the HAL switches the
 * frequency down whenever the CPU idles and back up as soon as there is
 * something to do.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

/* The HAL's power management (esp_pm_configure()), not NuttX's */

#include "esp_pm.h"
#include "esp_rom_sys.h"

#include "esp32s3_dfs.h"

#ifdef CONFIG_ESP32S3_DFS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_dfs_initialize
 *
 * Description:
 *   See esp32s3_dfs.h.
 *
 ****************************************************************************/

int esp32s3_dfs_initialize(void)
{
  const esp_pm_config_t config =
    {
      .max_freq_mhz       = CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ,
      .min_freq_mhz       = CONFIG_ESP32S3_DFS_MIN_FREQ_MHZ,
      .light_sleep_enable = false,
    };

  esp_err_t err;

  err = esp_pm_configure(&config);
  if (err != ESP_OK)
    {
      _err("ERROR: esp_pm_configure failed: %d\n", err);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Busy-wait for the given number of microseconds.  The ROM's delay counts
 *   CPU cycles at the current frequency, which the HAL updates at every
 *   switch, so it stays right as the frequency changes.
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  esp_rom_delay_us(microseconds);
}

#endif /* CONFIG_ESP32S3_DFS */
