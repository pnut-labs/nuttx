/****************************************************************************
 * drivers/power/battery/sy6970.c
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
 * Driver for the Silergy SY6970 single cell buck charger with power path.
 *
 * The register map follows the TI BQ25895 the part is modelled on; the
 * SY6970 identifies itself with revision 0 in register 0x14.  The driver
 * does not reset the charger, and changes only its watchdog and its input's
 * high impedance mode: it charges on its own with whatever it was last set
 * to, and the board decides what to change.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/sy6970.h>

#if defined(CONFIG_BATTERY_CHARGER) && defined(CONFIG_SY6970)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* REG00: input source control */

#define SY6970_REG00                 0x00
#define SY6970_EN_HIZ                (1 << 7)  /* Input disconnected */
#define SY6970_EN_ILIM               (1 << 6)  /* ILIM pin limits too */
#define SY6970_IINLIM_MASK           0x3f
#define SY6970_IINLIM_BASE           100       /* mA */
#define SY6970_IINLIM_STEP           50        /* mA */
#define SY6970_IINLIM_MAX            3250      /* mA */

/* REG04: fast charge current */

#define SY6970_REG04                 0x04
#define SY6970_ICHG_MASK             0x7f
#define SY6970_ICHG_STEP             64        /* mA */
#define SY6970_ICHG_MAX              5056      /* mA */

/* REG06: charge voltage */

#define SY6970_REG06                 0x06
#define SY6970_VREG_SHIFT            2
#define SY6970_VREG_MASK             (0x3f << SY6970_VREG_SHIFT)
#define SY6970_VREG_BASE             3840      /* mV */
#define SY6970_VREG_STEP             16        /* mV */
#define SY6970_VREG_MAX              4608      /* mV */

/* REG07: termination and timers */

#define SY6970_REG07                 0x07
#define SY6970_WATCHDOG_MASK         (3 << 4)  /* 00 = disabled */

/* REG09: BATFET */

#define SY6970_REG09                 0x09
#define SY6970_BATFET_DIS            (1 << 5)

/* REG0B: status */

#define SY6970_REG0B                 0x0b
#define SY6970_CHRG_STAT_SHIFT       3
#define SY6970_CHRG_STAT_MASK        (3 << SY6970_CHRG_STAT_SHIFT)
#define SY6970_CHRG_NONE             0
#define SY6970_CHRG_PRE              1
#define SY6970_CHRG_FAST             2
#define SY6970_CHRG_DONE             3
#define SY6970_PG_STAT               (1 << 2)  /* Input power good */

/* REG0C: faults.  The first read returns what has latched since the last
 * one and clears it.
 */

#define SY6970_REG0C                 0x0c
#define SY6970_WATCHDOG_FAULT        (1 << 7)
#define SY6970_CHRG_FAULT_SHIFT      4
#define SY6970_CHRG_FAULT_MASK       (3 << SY6970_CHRG_FAULT_SHIFT)
#define SY6970_CHRG_FAULT_INPUT      1
#define SY6970_CHRG_FAULT_THERMAL    2
#define SY6970_CHRG_FAULT_TIMER      3
#define SY6970_BAT_FAULT             (1 << 3)  /* Battery overvoltage */
#define SY6970_NTC_FAULT_MASK        0x07
#define SY6970_NTC_WARM              2
#define SY6970_NTC_COOL              3
#define SY6970_NTC_COLD              5
#define SY6970_NTC_HOT               6

/* REG14: part number and revision */

#define SY6970_REG14                 0x14
#define SY6970_ID_MASK               0x3f      /* PN, TS_PROFILE, DEV_REV */
#define SY6970_DEV_REV_MASK          0x03
#define SY6970_DEV_REV               0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sy6970_dev_s
{
  struct battery_charger_dev_s dev;  /* Battery charger device, must be first */

  FAR struct i2c_master_s *i2c;      /* I2C interface */
  uint8_t addr;                      /* I2C address */
  uint32_t frequency;                /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sy6970_getreg(FAR struct sy6970_dev_s *priv, uint8_t reg,
                         FAR uint8_t *value);
static int sy6970_putreg(FAR struct sy6970_dev_s *priv, uint8_t reg,
                         uint8_t value);
static int sy6970_modifyreg(FAR struct sy6970_dev_s *priv, uint8_t reg,
                            uint8_t clearbits, uint8_t setbits);

/* Battery driver lower half methods */

static int sy6970_state(FAR struct battery_charger_dev_s *dev,
                        FAR int *status);
static int sy6970_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *health);
static int sy6970_online(FAR struct battery_charger_dev_s *dev,
                         FAR bool *status);
static int sy6970_voltage(FAR struct battery_charger_dev_s *dev,
                          int value);
static int sy6970_current(FAR struct battery_charger_dev_s *dev,
                          int value);
static int sy6970_input_current(FAR struct battery_charger_dev_s *dev,
                                int value);
static int sy6970_operate(FAR struct battery_charger_dev_s *dev,
                          uintptr_t param);
static int sy6970_chipid(FAR struct battery_charger_dev_s *dev,
                         FAR unsigned int *value);
static int sy6970_get_voltage(FAR struct battery_charger_dev_s *dev,
                              FAR int *value);
static int sy6970_voltage_info(FAR struct battery_charger_dev_s *dev,
                               FAR int *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_sy6970ops =
{
  sy6970_state,
  sy6970_health,
  sy6970_online,
  sy6970_voltage,
  sy6970_current,
  sy6970_input_current,
  sy6970_operate,
  sy6970_chipid,
  sy6970_get_voltage,
  sy6970_voltage_info,
  NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sy6970_getreg
 ****************************************************************************/

static int sy6970_getreg(FAR struct sy6970_dev_s *priv, uint8_t reg,
                         FAR uint8_t *value)
{
  struct i2c_config_s config;
  int ret;

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_writeread(priv->i2c, &config, &reg, 1, value, 1);
  if (ret < 0)
    {
      baterr("ERROR: Read of 0x%02x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sy6970_putreg
 ****************************************************************************/

static int sy6970_putreg(FAR struct sy6970_dev_s *priv, uint8_t reg,
                         uint8_t value)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  buffer[0] = reg;
  buffer[1] = value;

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      baterr("ERROR: Write to 0x%02x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: sy6970_modifyreg
 ****************************************************************************/

static int sy6970_modifyreg(FAR struct sy6970_dev_s *priv, uint8_t reg,
                            uint8_t clearbits, uint8_t setbits)
{
  uint8_t value;
  int ret;

  ret = sy6970_getreg(priv, reg, &value);
  if (ret < 0)
    {
      return ret;
    }

  value = (value & ~clearbits) | setbits;
  return sy6970_putreg(priv, reg, value);
}

/****************************************************************************
 * Name: sy6970_state
 ****************************************************************************/

static int sy6970_state(FAR struct battery_charger_dev_s *dev,
                        FAR int *status)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;
  uint8_t value;
  int ret;

  ret = sy6970_getreg(priv, SY6970_REG0B, &value);
  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  switch ((value & SY6970_CHRG_STAT_MASK) >> SY6970_CHRG_STAT_SHIFT)
    {
      case SY6970_CHRG_PRE:
      case SY6970_CHRG_FAST:
        *status = BATTERY_CHARGING;
        break;

      case SY6970_CHRG_DONE:
        *status = BATTERY_FULL;
        break;

      default:
        *status = (value & SY6970_PG_STAT) != 0 ? BATTERY_IDLE :
                                                  BATTERY_DISCHARGING;
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: sy6970_health
 *
 * Description:
 *   Report the faults latched since the last call, most serious first.
 *
 ****************************************************************************/

static int sy6970_health(FAR struct battery_charger_dev_s *dev,
                         FAR int *health)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;
  uint8_t value;
  int ret;

  ret = sy6970_getreg(priv, SY6970_REG0C, &value);
  if (ret < 0)
    {
      *health = BATTERY_HEALTH_UNKNOWN;
      return ret;
    }

  if ((value & SY6970_BAT_FAULT) != 0)
    {
      *health = BATTERY_HEALTH_OVERVOLTAGE;
      return OK;
    }

  switch ((value & SY6970_CHRG_FAULT_MASK) >> SY6970_CHRG_FAULT_SHIFT)
    {
      case SY6970_CHRG_FAULT_THERMAL:
        *health = BATTERY_HEALTH_OVERHEAT;
        return OK;

      case SY6970_CHRG_FAULT_TIMER:
        *health = BATTERY_HEALTH_SAFE_TMR_EXP;
        return OK;

      case SY6970_CHRG_FAULT_INPUT:
        *health = BATTERY_HEALTH_UNSPEC_FAIL;
        return OK;

      default:
        break;
    }

  /* Warm and cool only slow charging down (JEITA); hot and cold stop it */

  if ((value & SY6970_NTC_FAULT_MASK) == SY6970_NTC_HOT)
    {
      *health = BATTERY_HEALTH_OVERHEAT;
    }
  else if ((value & SY6970_NTC_FAULT_MASK) == SY6970_NTC_COLD)
    {
      *health = BATTERY_HEALTH_COLD;
    }
  else if ((value & SY6970_WATCHDOG_FAULT) != 0)
    {
      *health = BATTERY_HEALTH_WD_TMR_EXP;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}

/****************************************************************************
 * Name: sy6970_online
 ****************************************************************************/

static int sy6970_online(FAR struct battery_charger_dev_s *dev,
                         FAR bool *status)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;
  uint8_t value;
  int ret;

  ret = sy6970_getreg(priv, SY6970_REG0B, &value);
  *status = ret >= 0 && (value & SY6970_PG_STAT) != 0;
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: sy6970_voltage
 ****************************************************************************/

static int sy6970_voltage(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;
  uint8_t steps;

  if (value < SY6970_VREG_BASE || value > SY6970_VREG_MAX)
    {
      return -EINVAL;
    }

  steps = (value - SY6970_VREG_BASE) / SY6970_VREG_STEP;
  return sy6970_modifyreg(priv, SY6970_REG06, SY6970_VREG_MASK,
                          steps << SY6970_VREG_SHIFT);
}

/****************************************************************************
 * Name: sy6970_current
 ****************************************************************************/

static int sy6970_current(FAR struct battery_charger_dev_s *dev, int value)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;

  if (value < 0 || value > SY6970_ICHG_MAX)
    {
      return -EINVAL;
    }

  return sy6970_modifyreg(priv, SY6970_REG04, SY6970_ICHG_MASK,
                          value / SY6970_ICHG_STEP);
}

/****************************************************************************
 * Name: sy6970_input_current
 *
 * Description:
 *   Set the input current limit.  The limit of the board's ILIM resistor
 *   applies as well while EN_ILIM is set, as it is by default; the driver
 *   never clears it, and BATTERY_INPUT_CURRENT_EXT_LIM sets it again.  The
 *   charger sets its own limit again each time it identifies a newly
 *   plugged adapter.
 *
 ****************************************************************************/

static int sy6970_input_current(FAR struct battery_charger_dev_s *dev,
                                int value)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;

  if (value == BATTERY_INPUT_CURRENT_EXT_LIM)
    {
      return sy6970_modifyreg(priv, SY6970_REG00, 0, SY6970_EN_ILIM);
    }

  if (value < SY6970_IINLIM_BASE || value > SY6970_IINLIM_MAX)
    {
      return -EINVAL;
    }

  return sy6970_modifyreg(priv, SY6970_REG00, SY6970_IINLIM_MASK,
                          (value - SY6970_IINLIM_BASE) /
                          SY6970_IINLIM_STEP);
}

/****************************************************************************
 * Name: sy6970_operate
 ****************************************************************************/

static int sy6970_operate(FAR struct battery_charger_dev_s *dev,
                          uintptr_t param)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;
  FAR struct batio_operate_msg_s *msg =
    (FAR struct batio_operate_msg_s *)param;

  switch (msg->operate_type)
    {
      case BATIO_OPRTN_SHIPMODE:
      case BATIO_OPRTN_SYSOFF:
        return sy6970_modifyreg(priv, SY6970_REG09, 0, SY6970_BATFET_DIS);

      case BATIO_OPRTN_SYSON:
        return sy6970_modifyreg(priv, SY6970_REG09, SY6970_BATFET_DIS, 0);

      default:
        return -ENOSYS;
    }
}

/****************************************************************************
 * Name: sy6970_chipid
 ****************************************************************************/

static int sy6970_chipid(FAR struct battery_charger_dev_s *dev,
                         FAR unsigned int *value)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;
  uint8_t regval;
  int ret;

  ret = sy6970_getreg(priv, SY6970_REG14, &regval);
  if (ret >= 0)
    {
      *value = regval & SY6970_ID_MASK;
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: sy6970_get_voltage
 ****************************************************************************/

static int sy6970_get_voltage(FAR struct battery_charger_dev_s *dev,
                              FAR int *value)
{
  FAR struct sy6970_dev_s *priv = (FAR struct sy6970_dev_s *)dev;
  uint8_t regval;
  int ret;

  ret = sy6970_getreg(priv, SY6970_REG06, &regval);
  if (ret >= 0)
    {
      *value = SY6970_VREG_BASE + SY6970_VREG_STEP *
               ((regval & SY6970_VREG_MASK) >> SY6970_VREG_SHIFT);
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: sy6970_voltage_info
 ****************************************************************************/

static int sy6970_voltage_info(FAR struct battery_charger_dev_s *dev,
                               FAR int *value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sy6970_initialize
 *
 * Description:
 *   Initialize the SY6970 battery charger.  See
 *   include/nuttx/power/sy6970.h.
 *
 ****************************************************************************/

FAR struct battery_charger_dev_s *
sy6970_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                  uint32_t frequency)
{
  FAR struct sy6970_dev_s *priv;
  uint8_t id;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct sy6970_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops   = &g_sy6970ops;
  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->frequency = frequency;

  ret = sy6970_getreg(priv, SY6970_REG14, &id);
  if (ret < 0 || (id & SY6970_DEV_REV_MASK) != SY6970_DEV_REV)
    {
      baterr("ERROR: No SY6970 at 0x%02x (%d, id 0x%02x)\n",
             addr, ret, ret < 0 ? 0 : id);
      kmm_free(priv);
      return NULL;
    }

  /* A failed write below leaves a charger that still charges and reports,
   * which is worth more than none
   */

  ret = sy6970_modifyreg(priv, SY6970_REG07, SY6970_WATCHDOG_MASK, 0);
  if (ret < 0)
    {
      baterr("ERROR: Failed to disable the watchdog: %d\n", ret);
    }

  /* The charger keeps its registers through a reset of the host, and with
   * its input left in high impedance mode it neither charges nor powers
   * the system from USB, while reporting no input.  Start with the input
   * connected.
   */

  ret = sy6970_modifyreg(priv, SY6970_REG00, SY6970_EN_HIZ, 0);
  if (ret < 0)
    {
      baterr("ERROR: Failed to connect the input: %d\n", ret);
    }

  return &priv->dev;
}

#endif /* CONFIG_BATTERY_CHARGER && CONFIG_SY6970 */
