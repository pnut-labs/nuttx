/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_battery.c
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
 * The cell's fuel gauge (BQ27220) and charger (SY6970), both on the I2C
 * bus.  Both are powered by the cell rather than by the board, so they keep
 * their settings across the board's resets, but not across the cell being
 * disconnected; the settings are therefore applied at every start.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <arch/board/board.h>

#ifdef CONFIG_BQ27220
#  include <nuttx/power/battery_gauge.h>
#  include <nuttx/power/bq27220.h>
#endif

#ifdef CONFIG_SY6970
#  include <nuttx/power/battery_charger.h>
#  include <nuttx/power/battery_ioctl.h>
#  include <nuttx/power/sy6970.h>
#endif

#include "lilygo-tdeck-max.h"

#if defined(CONFIG_BQ27220) || defined(CONFIG_SY6970)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TDECK_BATTERY_I2C_FREQUENCY 400000

/* After cutting the cell off, how long a board that is still running waits
 * before it takes the cell back
 */

#define TDECK_SHIPMODE_WAIT_MS      100

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SY6970
static FAR struct battery_charger_dev_s *g_charger;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_gauge_initialize
 *
 * Description:
 *   Register the fuel gauge as /dev/batt0.  The gauge is told the cell's
 *   capacity, which is written only when it holds another: a couple of
 *   seconds, once.
 *
 ****************************************************************************/

#ifdef CONFIG_BQ27220
static int tdeck_gauge_initialize(FAR struct i2c_master_s *i2c)
{
  FAR struct battery_gauge_dev_s *gauge;

  gauge = bq27220_initialize(i2c, BQ27220_I2C_ADDRESS,
                             TDECK_BATTERY_I2C_FREQUENCY,
                             BOARD_BATTERY_MAH);
  if (gauge == NULL)
    {
      return -ENODEV;
    }

  return battery_gauge_register("/dev/batt0", gauge);
}
#endif

/****************************************************************************
 * Name: tdeck_charger_initialize
 *
 * Description:
 *   Register the charger as /dev/charger0, after setting its charge
 *   voltage and current.
 *
 ****************************************************************************/

#ifdef CONFIG_SY6970
static int tdeck_charger_initialize(FAR struct i2c_master_s *i2c)
{
  FAR struct battery_charger_dev_s *charger;
  int ret;

  charger = sy6970_initialize(i2c, SY6970_I2C_ADDRESS,
                              TDECK_BATTERY_I2C_FREQUENCY);
  if (charger == NULL)
    {
      return -ENODEV;
    }

  ret = charger->ops->voltage(charger, BOARD_CHARGE_MV);
  if (ret >= 0)
    {
      ret = charger->ops->current(charger, BOARD_CHARGE_MA);
    }

  if (ret < 0)
    {
      /* Still register it: it charges with its previous settings, and its
       * status is worth having.
       */

      syslog(LOG_ERR, "ERROR: Failed to configure the charger: %d\n", ret);
    }

  g_charger = charger;
  return battery_charger_register("/dev/charger0", charger);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_battery_initialize
 *
 * Description:
 *   Register the fuel gauge as /dev/batt0 and the charger as
 *   /dev/charger0.  A failure of one does not stop the other.
 *
 ****************************************************************************/

int tdeck_battery_initialize(FAR struct i2c_master_s *i2c)
{
  int result = OK;
  int ret;

#ifdef CONFIG_BQ27220
  ret = tdeck_gauge_initialize(i2c);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Fuel gauge: %d\n", ret);
      result = ret;
    }
#endif

#ifdef CONFIG_SY6970
  ret = tdeck_charger_initialize(i2c);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Charger: %d\n", ret);
      result = ret;
    }
#endif

  return result;
}

/****************************************************************************
 * Name: board_power_off
 *
 * Description:
 *   Power the board off, through BOARDIOC_POWEROFF: the charger cuts the
 *   cell off (ship mode), which leaves nothing powered but the cell's own
 *   chips.  Plugging in USB power turns the board on again.
 *
 *   On USB power the board cannot be turned off: it would stay on, and the
 *   cell, cut off, would not charge.  The charger's lock is held throughout,
 *   so that no other call reaches the charger in between.
 *
 * Returned Value:
 *   Does not return when the board is off.  -EBUSY on USB power, -ENODEV
 *   without the charger, or the error that kept the charger from cutting
 *   the cell off.
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL_POWEROFF) && defined(CONFIG_SY6970)
int board_power_off(int status)
{
  struct batio_operate_msg_s msg;
  bool online;
  int ret;

  if (g_charger == NULL)
    {
      return -ENODEV;
    }

  ret = nxmutex_lock(&g_charger->batlock);
  if (ret < 0)
    {
      return ret;
    }

  ret = g_charger->ops->online(g_charger, &online);
  if (ret >= 0 && online)
    {
      ret = -EBUSY;
    }

  if (ret < 0)
    {
      nxmutex_unlock(&g_charger->batlock);
      return ret;
    }

  msg.operate_type = BATIO_OPRTN_SHIPMODE;
  ret = g_charger->ops->operate(g_charger, (uintptr_t)&msg);
  if (ret < 0)
    {
      nxmutex_unlock(&g_charger->batlock);
      return ret;
    }

  /* The board is off now, unless USB power came in meanwhile.  Then take
   * the cell back, so that it charges.
   */

  up_mdelay(TDECK_SHIPMODE_WAIT_MS);

  msg.operate_type = BATIO_OPRTN_SYSON;
  ret = g_charger->ops->operate(g_charger, (uintptr_t)&msg);
  nxmutex_unlock(&g_charger->batlock);

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to connect the cell again: %d\n", ret);
      return ret;
    }

  return -EBUSY;
}
#endif

#endif /* CONFIG_BQ27220 || CONFIG_SY6970 */
