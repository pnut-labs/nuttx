/****************************************************************************
 * drivers/power/battery/bq27220.c
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
 * Driver for the TI BQ27220 single cell CEDV battery fuel gauge.
 *
 * The gauge is read through its standard commands, which work while it is
 * sealed.  Its data memory is only touched to set the battery capacity, and
 * only when it differs from what the board asks for: the gauge keeps its
 * configuration in RAM, powered by the cell, so a mismatch normally means
 * the battery was disconnected or the gauge has never been configured.
 *
 * The data memory sequence, and the delays in it, follow the driver the
 * Flipper Zero ships for the same part; the delays are what that driver
 * found the gauge to need, with margin.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/battery_ioctl.h>
#include <nuttx/power/bq27220.h>

#if defined(CONFIG_BATTERY_GAUGE) && defined(CONFIG_BQ27220)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Standard commands.  Each is a 16-bit little endian register. */

#define BQ27220_CMD_CONTROL          0x00
#define BQ27220_CMD_TEMPERATURE      0x06  /* 0.1 K */
#define BQ27220_CMD_VOLTAGE          0x08  /* mV */
#define BQ27220_CMD_BATTERYSTATUS    0x0a
#define BQ27220_CMD_CURRENT          0x0c  /* mA, signed */
#define BQ27220_CMD_STATEOFCHARGE    0x2c  /* % */
#define BQ27220_CMD_OPERATIONSTATUS  0x3a
#define BQ27220_CMD_DESIGNCAPACITY   0x3c  /* mAh */
#define BQ27220_CMD_MACSUBCMD        0x3e  /* Data memory address */
#define BQ27220_CMD_MACDATA          0x40  /* Data memory contents */
#define BQ27220_CMD_MACDATASUM       0x60  /* Checksum, then length */

/* Control subcommands */

#define BQ27220_CTRL_DEVICE_NUMBER   0x0001
#define BQ27220_CTRL_SEALED          0x0030
#define BQ27220_CTRL_ENTER_CFGUPDATE 0x0090
#define BQ27220_CTRL_EXIT_CFGUPDATE  0x0091  /* ...and reinitialise */

/* Default keys.  They are documented, so they only keep out accidents. */

#define BQ27220_KEY_UNSEAL1          0x0414
#define BQ27220_KEY_UNSEAL2          0x3672
#define BQ27220_KEY_FULLACCESS       0xffff

#define BQ27220_DEVICE_NUMBER        0x0220

/* BatteryStatus() bits */

#define BQ27220_BS_DSG               (1 << 0)  /* Discharging */
#define BQ27220_BS_BATTPRES          (1 << 3)  /* Battery present */
#define BQ27220_BS_FC                (1 << 9)  /* Fully charged */

/* OperationStatus() bits */

#define BQ27220_OS_SEC_SHIFT         1
#define BQ27220_OS_SEC_MASK          (3 << BQ27220_OS_SEC_SHIFT)
#define BQ27220_OS_SEC_FULL          1
#define BQ27220_OS_SEC_UNSEALED      2
#define BQ27220_OS_SEC_SEALED        3
#define BQ27220_OS_CFGUPDATE         (1 << 10)

#define BQ27220_SEC(os) \
  (((os) & BQ27220_OS_SEC_MASK) >> BQ27220_OS_SEC_SHIFT)

/* Data memory: CEDV profile 1, big endian */

#define BQ27220_DM_FULLCHARGECAP     0x929d
#define BQ27220_DM_DESIGNCAP         0x929f

/* Delays, in microseconds */

#define BQ27220_KEY_DELAY            5000     /* Between key writes */
#define BQ27220_MAC_DELAY            1000     /* Address to data valid */
#define BQ27220_DM_WRITE_DELAY       250      /* Data to checksum */
#define BQ27220_DM_COMMIT_DELAY      10000    /* After a checksum */
#define BQ27220_CFG_APPLY_DELAY      2000000  /* After leaving CFGUPDATE */
#define BQ27220_POLL_DELAY           1000
#define BQ27220_POLL_TIMEOUT         2000000

/* Current below this, either way, is taken as none, so that measurement
 * noise around zero does not make the state flicker.
 */

#define BQ27220_IDLE_CURRENT         10  /* mA */

/* Temperature() is in tenths of a Kelvin */

#define BQ27220_ZERO_CELSIUS         2731

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bq27220_dev_s
{
  struct battery_gauge_dev_s dev;  /* Battery gauge device, must be first */

  FAR struct i2c_master_s *i2c;    /* I2C interface */
  uint8_t addr;                    /* I2C address */
  uint32_t frequency;              /* I2C frequency */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bq27220_read(FAR struct bq27220_dev_s *priv, uint8_t cmd,
                        FAR uint8_t *buffer, int len);
static int bq27220_write(FAR struct bq27220_dev_s *priv, uint8_t cmd,
                         FAR const uint8_t *data, int len);
static int bq27220_getword(FAR struct bq27220_dev_s *priv, uint8_t cmd,
                           FAR uint16_t *value);
static int bq27220_control(FAR struct bq27220_dev_s *priv, uint16_t subcmd);
static int bq27220_waitstatus(FAR struct bq27220_dev_s *priv,
                              uint16_t mask, uint16_t value);
static int bq27220_setaccess(FAR struct bq27220_dev_s *priv, int sec);
static int bq27220_dmwrite16(FAR struct bq27220_dev_s *priv,
                             uint16_t address, uint16_t value);
static int bq27220_leavecfg(FAR struct bq27220_dev_s *priv);
static int bq27220_provision(FAR struct bq27220_dev_s *priv,
                             uint16_t capacity);

/* Battery driver lower half methods */

static int bq27220_state(FAR struct battery_gauge_dev_s *dev,
                         FAR int *status);
static int bq27220_online(FAR struct battery_gauge_dev_s *dev,
                          FAR bool *status);
static int bq27220_voltage(FAR struct battery_gauge_dev_s *dev,
                           FAR int *value);
static int bq27220_capacity(FAR struct battery_gauge_dev_s *dev,
                            FAR int *value);
static int bq27220_current(FAR struct battery_gauge_dev_s *dev,
                           FAR int *value);
static int bq27220_temp(FAR struct battery_gauge_dev_s *dev,
                        FAR int *value);
static int bq27220_chipid(FAR struct battery_gauge_dev_s *dev,
                          FAR unsigned int *value);
static int bq27220_operate(FAR struct battery_gauge_dev_s *dev,
                           FAR int *param);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_gauge_operations_s g_bq27220ops =
{
  bq27220_state,
  bq27220_online,
  bq27220_voltage,
  bq27220_capacity,
  bq27220_current,
  bq27220_temp,
  bq27220_chipid,
  bq27220_operate,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq27220_read
 *
 * Description:
 *   Read len bytes starting at a command register.
 *
 ****************************************************************************/

static int bq27220_read(FAR struct bq27220_dev_s *priv, uint8_t cmd,
                        FAR uint8_t *buffer, int len)
{
  struct i2c_config_s config;
  int ret;

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  ret = i2c_writeread(priv->i2c, &config, &cmd, 1, buffer, len);
  if (ret < 0)
    {
      baterr("ERROR: Read of 0x%02x failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: bq27220_write
 *
 * Description:
 *   Write up to four bytes starting at a command register.
 *
 ****************************************************************************/

static int bq27220_write(FAR struct bq27220_dev_s *priv, uint8_t cmd,
                         FAR const uint8_t *data, int len)
{
  struct i2c_config_s config;
  uint8_t buffer[5];
  int ret;

  DEBUGASSERT(len > 0 && len < (int)sizeof(buffer));

  config.frequency = priv->frequency;
  config.address   = priv->addr;
  config.addrlen   = 7;

  buffer[0] = cmd;
  memcpy(&buffer[1], data, len);

  ret = i2c_write(priv->i2c, &config, buffer, len + 1);
  if (ret < 0)
    {
      baterr("ERROR: Write to 0x%02x failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: bq27220_getword
 ****************************************************************************/

static int bq27220_getword(FAR struct bq27220_dev_s *priv, uint8_t cmd,
                           FAR uint16_t *value)
{
  uint8_t buffer[2];
  int ret;

  ret = bq27220_read(priv, cmd, buffer, sizeof(buffer));
  if (ret >= 0)
    {
      *value = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
    }

  return ret;
}

/****************************************************************************
 * Name: bq27220_control
 *
 * Description:
 *   Issue a Control() subcommand.
 *
 ****************************************************************************/

static int bq27220_control(FAR struct bq27220_dev_s *priv, uint16_t subcmd)
{
  uint8_t data[2];

  data[0] = subcmd & 0xff;
  data[1] = subcmd >> 8;

  return bq27220_write(priv, BQ27220_CMD_CONTROL, data, sizeof(data));
}

/****************************************************************************
 * Name: bq27220_waitstatus
 *
 * Description:
 *   Wait until the masked bits of OperationStatus() equal value.
 *
 ****************************************************************************/

static int bq27220_waitstatus(FAR struct bq27220_dev_s *priv,
                              uint16_t mask, uint16_t value)
{
  uint16_t status;
  int waited;
  int ret;

  for (waited = 0; waited < BQ27220_POLL_TIMEOUT;
       waited += BQ27220_POLL_DELAY)
    {
      ret = bq27220_getword(priv, BQ27220_CMD_OPERATIONSTATUS, &status);
      if (ret >= 0 && (status & mask) == value)
        {
          return OK;
        }

      nxsched_usleep(BQ27220_POLL_DELAY);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: bq27220_setaccess
 *
 * Description:
 *   Move the gauge to a security level: sealed, unsealed or full access.
 *   Going up takes the keys, one level at a time.
 *
 ****************************************************************************/

static int bq27220_setaccess(FAR struct bq27220_dev_s *priv, int sec)
{
  uint16_t status;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_OPERATIONSTATUS, &status);
  if (ret < 0)
    {
      return ret;
    }

  if (BQ27220_SEC(status) == sec)
    {
      return OK;
    }

  if (sec == BQ27220_OS_SEC_SEALED)
    {
      ret = bq27220_control(priv, BQ27220_CTRL_SEALED);
      nxsched_usleep(BQ27220_MAC_DELAY);
    }
  else
    {
      if (BQ27220_SEC(status) == BQ27220_OS_SEC_SEALED)
        {
          bq27220_control(priv, BQ27220_KEY_UNSEAL1);
          nxsched_usleep(BQ27220_KEY_DELAY);
          bq27220_control(priv, BQ27220_KEY_UNSEAL2);
          nxsched_usleep(BQ27220_KEY_DELAY);
        }

      if (sec == BQ27220_OS_SEC_FULL)
        {
          bq27220_control(priv, BQ27220_KEY_FULLACCESS);
          nxsched_usleep(BQ27220_KEY_DELAY);
          bq27220_control(priv, BQ27220_KEY_FULLACCESS);
          nxsched_usleep(BQ27220_KEY_DELAY);
        }
    }

  ret = bq27220_getword(priv, BQ27220_CMD_OPERATIONSTATUS, &status);
  if (ret < 0)
    {
      return ret;
    }

  if (BQ27220_SEC(status) != sec)
    {
      baterr("ERROR: Security level %d, wanted %d\n",
             BQ27220_SEC(status), sec);
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: bq27220_dmwrite16
 *
 * Description:
 *   Write a 16-bit data memory parameter.  The gauge must be in
 *   configuration update mode.  The address goes little endian and the
 *   value big endian, followed by a checksum over both and the length of
 *   the whole transfer.
 *
 ****************************************************************************/

static int bq27220_dmwrite16(FAR struct bq27220_dev_s *priv,
                             uint16_t address, uint16_t value)
{
  uint8_t data[4];
  uint8_t sum[2];
  int ret;

  data[0] = address & 0xff;
  data[1] = address >> 8;
  data[2] = value >> 8;
  data[3] = value & 0xff;

  ret = bq27220_write(priv, BQ27220_CMD_MACSUBCMD, data, sizeof(data));
  if (ret < 0)
    {
      return ret;
    }

  nxsched_usleep(BQ27220_DM_WRITE_DELAY);

  sum[0] = ~(uint8_t)(data[0] + data[1] + data[2] + data[3]);
  sum[1] = sizeof(data) + sizeof(sum);

  ret = bq27220_write(priv, BQ27220_CMD_MACDATASUM, sum, sizeof(sum));
  nxsched_usleep(BQ27220_DM_COMMIT_DELAY);
  return ret;
}

/****************************************************************************
 * Name: bq27220_leavecfg
 *
 * Description:
 *   Leave configuration update mode, applying what was written, and wait
 *   for the gauge to be out of it.
 *
 ****************************************************************************/

static int bq27220_leavecfg(FAR struct bq27220_dev_s *priv)
{
  int ret;

  ret = bq27220_control(priv, BQ27220_CTRL_EXIT_CFGUPDATE);
  nxsched_usleep(BQ27220_CFG_APPLY_DELAY);

  if (ret >= 0)
    {
      ret = bq27220_waitstatus(priv, BQ27220_OS_CFGUPDATE, 0);
    }

  if (ret < 0)
    {
      baterr("ERROR: Gauge did not leave configuration update: %d\n",
             ret);
    }

  return ret;
}

/****************************************************************************
 * Name: bq27220_provision
 *
 * Description:
 *   Make the gauge's design capacity match the battery.  DesignCapacity()
 *   reads it without unsealing, so the common case, where it is already
 *   right, costs one read.  It reads the active CEDV profile, and the
 *   writes go to profile 1, the default: with another profile selected,
 *   the capacity would be written again at every start.
 *
 ****************************************************************************/

static int bq27220_provision(FAR struct bq27220_dev_s *priv,
                             uint16_t capacity)
{
  static const uint8_t enter[2] =
    {
      BQ27220_CTRL_ENTER_CFGUPDATE & 0xff, BQ27220_CTRL_ENTER_CFGUPDATE >> 8
    };

  bool entering = false;
  uint16_t current;
  int err;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_DESIGNCAPACITY, &current);
  if (ret < 0)
    {
      return ret;
    }

  if (current == capacity)
    {
      return OK;
    }

  batinfo("Design capacity %u mAh, setting %u mAh\n", current, capacity);

  ret = bq27220_setaccess(priv, BQ27220_OS_SEC_FULL);
  if (ret < 0)
    {
      goto errout;
    }

  /* The Flipper Zero's driver enters configuration update through the data
   * memory address register rather than Control(); either is accepted.
   */

  ret = bq27220_write(priv, BQ27220_CMD_MACSUBCMD, enter, sizeof(enter));
  if (ret >= 0)
    {
      entering = true;
      ret = bq27220_waitstatus(priv, BQ27220_OS_CFGUPDATE,
                               BQ27220_OS_CFGUPDATE);
    }

  if (ret < 0)
    {
      baterr("ERROR: Gauge did not enter configuration update: %d\n", ret);
      goto errout;
    }

  /* The initial full charge capacity is the gauge's starting point until
   * it has learned the real one, so it gets the same value.
   */

  ret = bq27220_dmwrite16(priv, BQ27220_DM_FULLCHARGECAP, capacity);
  if (ret >= 0)
    {
      ret = bq27220_dmwrite16(priv, BQ27220_DM_DESIGNCAP, capacity);
    }

errout:

  /* Leave configuration update even after a failed write, or after a wait
   * that gave up before the gauge entered it, or the gauge stops gauging
   * until it is reset.
   */

  if (entering)
    {
      err = bq27220_leavecfg(priv);
      if (ret >= 0)
        {
          ret = err;
        }
    }

  bq27220_setaccess(priv, BQ27220_OS_SEC_SEALED);

  if (ret >= 0)
    {
      ret = bq27220_getword(priv, BQ27220_CMD_DESIGNCAPACITY, &current);
      if (ret >= 0 && current != capacity)
        {
          baterr("ERROR: Design capacity reads back %u mAh\n", current);
          ret = -EIO;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bq27220_state
 *
 * Description:
 *   Report whether the battery is charging, discharging, full or idle.
 *
 ****************************************************************************/

static int bq27220_state(FAR struct battery_gauge_dev_s *dev,
                         FAR int *status)
{
  FAR struct bq27220_dev_s *priv = (FAR struct bq27220_dev_s *)dev;
  uint16_t battstatus;
  uint16_t current;
  uint16_t soc;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_BATTERYSTATUS, &battstatus);
  if (ret >= 0)
    {
      ret = bq27220_getword(priv, BQ27220_CMD_CURRENT, &current);
    }

  if (ret >= 0)
    {
      ret = bq27220_getword(priv, BQ27220_CMD_STATEOFCHARGE, &soc);
    }

  if (ret < 0)
    {
      *status = BATTERY_UNKNOWN;
      return ret;
    }

  /* The gauge only sets DSG above its own discharge threshold, which a
   * lightly loaded system can stay under, so the current decides as well.
   * A battery that is topped up and resting on external power reports
   * neither DSG nor FC, and no current, so a full state of charge is taken
   * as full too.
   */

  if ((battstatus & BQ27220_BS_DSG) != 0 ||
      (int16_t)current <= -BQ27220_IDLE_CURRENT)
    {
      *status = BATTERY_DISCHARGING;
    }
  else if ((int16_t)current >= BQ27220_IDLE_CURRENT &&
           (battstatus & BQ27220_BS_FC) == 0)
    {
      *status = BATTERY_CHARGING;
    }
  else if ((battstatus & BQ27220_BS_FC) != 0 || soc >= 100)
    {
      *status = BATTERY_FULL;
    }
  else
    {
      *status = BATTERY_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: bq27220_online
 ****************************************************************************/

static int bq27220_online(FAR struct battery_gauge_dev_s *dev,
                          FAR bool *status)
{
  FAR struct bq27220_dev_s *priv = (FAR struct bq27220_dev_s *)dev;
  uint16_t battstatus;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_BATTERYSTATUS, &battstatus);
  *status = ret >= 0 && (battstatus & BQ27220_BS_BATTPRES) != 0;
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: bq27220_voltage
 ****************************************************************************/

static int bq27220_voltage(FAR struct battery_gauge_dev_s *dev,
                           FAR int *value)
{
  FAR struct bq27220_dev_s *priv = (FAR struct bq27220_dev_s *)dev;
  uint16_t regval;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_VOLTAGE, &regval);
  if (ret >= 0)
    {
      *value = regval;
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: bq27220_capacity
 ****************************************************************************/

static int bq27220_capacity(FAR struct battery_gauge_dev_s *dev,
                            FAR int *value)
{
  FAR struct bq27220_dev_s *priv = (FAR struct bq27220_dev_s *)dev;
  uint16_t regval;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_STATEOFCHARGE, &regval);
  if (ret >= 0)
    {
      *value = regval;
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: bq27220_current
 ****************************************************************************/

static int bq27220_current(FAR struct battery_gauge_dev_s *dev,
                           FAR int *value)
{
  FAR struct bq27220_dev_s *priv = (FAR struct bq27220_dev_s *)dev;
  uint16_t regval;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_CURRENT, &regval);
  if (ret >= 0)
    {
      *value = (int16_t)regval;
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: bq27220_temp
 ****************************************************************************/

static int bq27220_temp(FAR struct battery_gauge_dev_s *dev,
                        FAR int *value)
{
  FAR struct bq27220_dev_s *priv = (FAR struct bq27220_dev_s *)dev;
  uint16_t regval;
  int ret;

  ret = bq27220_getword(priv, BQ27220_CMD_TEMPERATURE, &regval);
  if (ret >= 0)
    {
      *value = (int)regval - BQ27220_ZERO_CELSIUS;
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: bq27220_chipid
 *
 * Description:
 *   Read the device number, which is 0x0220 for this part.  It is a
 *   subcommand whose answer appears in the data memory window.
 *
 ****************************************************************************/

static int bq27220_chipid(FAR struct battery_gauge_dev_s *dev,
                          FAR unsigned int *value)
{
  FAR struct bq27220_dev_s *priv = (FAR struct bq27220_dev_s *)dev;
  uint8_t buffer[2];
  int ret;

  ret = bq27220_control(priv, BQ27220_CTRL_DEVICE_NUMBER);
  if (ret < 0)
    {
      return ret;
    }

  nxsched_usleep(BQ27220_MAC_DELAY);

  ret = bq27220_read(priv, BQ27220_CMD_MACDATA, buffer, sizeof(buffer));
  if (ret >= 0)
    {
      *value = (unsigned int)buffer[0] | ((unsigned int)buffer[1] << 8);
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: bq27220_operate
 ****************************************************************************/

static int bq27220_operate(FAR struct battery_gauge_dev_s *dev,
                           FAR int *param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bq27220_initialize
 *
 * Description:
 *   Initialize the BQ27220 battery fuel gauge.  See
 *   include/nuttx/power/bq27220.h.
 *
 ****************************************************************************/

FAR struct battery_gauge_dev_s *
bq27220_initialize(FAR struct i2c_master_s *i2c, uint8_t addr,
                   uint32_t frequency, uint16_t capacity)
{
  FAR struct bq27220_dev_s *priv;
  unsigned int devnum;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct bq27220_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops   = &g_bq27220ops;
  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->frequency = frequency;

  ret = bq27220_chipid(&priv->dev, &devnum);
  if (ret < 0 || devnum != BQ27220_DEVICE_NUMBER)
    {
      baterr("ERROR: No BQ27220 at 0x%02x (%d, device 0x%04x)\n",
             addr, ret, ret < 0 ? 0 : devnum);
      kmm_free(priv);
      return NULL;
    }

  /* A failed update leaves the gauge working with its old capacity, which
   * is worth more than no gauge at all.
   */

  if (capacity != 0)
    {
      ret = bq27220_provision(priv, capacity);
      if (ret < 0)
        {
          baterr("ERROR: Failed to set the capacity: %d\n", ret);
        }
    }

  return &priv->dev;
}

#endif /* CONFIG_BATTERY_GAUGE && CONFIG_BQ27220 */
