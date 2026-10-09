/****************************************************************************
 * drivers/input/cst3530.c
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

/* Driver for the Hynitron CST3530 capacitive touch controller, and the
 * CST66xx parts that speak its protocol: Hynitron's own driver handles
 * them as one family.
 *
 * Register addresses are 32 bits, sent big endian, and writing an address
 * with no data is how the controller takes its commands.  Each transfer is
 * a write of the address, a stop, and a separate read.  The controller's
 * I2C interface can be in a low power state in which the first transfer
 * only wakes it, so the wake command is always sent twice.
 *
 * A report is read from 0xd0070000:
 *
 *   0, 1   checksum: 0x55 plus the sum of the record bytes, little endian
 *   2      report type, 0xff for touches and touch keys
 *   3      touch records (low nibble) and touch key records (high nibble)
 *   4...   the records, 5 bytes each, touch keys first
 *
 * Only the first record comes with the address; the rest are read by
 * carrying on with a plain read.  A record is
 *
 *   0      X, bits 7-0
 *   1      Y, bits 7-0
 *   2      pressure
 *   3      X, bits 11-8 (low nibble); Y, bits 11-8 (high nibble)
 *   4      the touch's id (low nibble); non-zero while pressed (high)
 *
 * A touch key's record carries the key's id in byte 4, like a touch.
 * Writing 0xd00002ab acknowledges the report.
 *
 * The interrupt hands over to a worker, which reads the report and passes
 * on what changed: touch keys pressed and released, touches that came or
 * moved, and a release for every touch no longer reported.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/cst3530.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/sched.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_INPUT_CST3530

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error "The CST3530 driver needs the high priority work queue"
#endif

/* Commands, and the registers read */

#define CST3530_CMD_WAKE        0xd0000400  /* Leave low power I2C */
#define CST3530_CMD_NORMAL1     0xd0000000  /* These three: normal mode */
#define CST3530_CMD_NORMAL2     0xd0000c00
#define CST3530_CMD_NORMAL3     0xd0000100
#define CST3530_CMD_ACK         0xd00002ab  /* Report read */
#define CST3530_CMD_DEEPSLEEP   0xd00022ab

#define CST3530_REG_INFO        0xd0030000
#define CST3530_REG_REPORT      0xd0070000

/* The information block, valid in normal mode, starts with a signature
 * that tells this family from other Hynitron parts at the same address
 */

#define CST3530_INFO_SIZE       50
#define CST3530_INFO_MAGIC      0xca        /* Bytes 2 and 3 */
#define CST3530_INFO_TRIES      4

/* Reports */

#define CST3530_REPORT_TOUCH    0xff
#define CST3530_HEADER_SIZE     4
#define CST3530_RECORD_SIZE     5
#define CST3530_MAX_RECORDS     5           /* Touches and keys together */
#define CST3530_CHECKSUM_SEED   0x55
#define CST3530_MAX_IDS         16          /* Ids are a nibble */

/* Timing, from Hynitron's driver */

#define CST3530_RESET_US        8000        /* The reset pulse */
#define CST3530_BOOT_US         50000       /* From reset to ready */
#define CST3530_WAKE_US         1000        /* Between the two wakes */
#define CST3530_INFO_RETRY_US   10000       /* Between info attempts */

/* After a bus error, the worker tries again this much later, with the
 * interrupt masked meanwhile
 */

#define CST3530_RETRY           MSEC2TICK(100)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cst3530_dev_s
{
  struct touch_lowerhalf_s lower;     /* Must be first */
  struct keyboard_lowerhalf_s tkeys;  /* The touch keys, if any */

  FAR struct i2c_master_s *i2c;
  FAR const struct cst3530_config_s *config;
  struct i2c_config_s i2cconfig;

  mutex_t lock;                       /* Guards all below */
  struct work_s work;                 /* Reads a report */
  int nopen;                          /* Files open; awake while any */
  uint16_t down;                      /* Touch ids pressed */
  uint16_t tdown;                     /* Touch key ids pressed */
  int16_t lastx[CST3530_MAX_IDS];     /* Each id's last position */
  int16_t lasty[CST3530_MAX_IDS];
  uint8_t lastp[CST3530_MAX_IDS];     /* And its last pressure */

  uint8_t sample[SIZEOF_TOUCH_SAMPLE_S(CST3530_MAX_RECORDS)];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cst3530_command(FAR struct cst3530_dev_s *priv, uint32_t cmd);
static int cst3530_read(FAR struct cst3530_dev_s *priv, uint32_t reg,
                        FAR uint8_t *buffer, size_t len);
static int cst3530_normal(FAR struct cst3530_dev_s *priv);
static void cst3530_sleep(FAR struct cst3530_dev_s *priv);
static int cst3530_reset(FAR struct cst3530_dev_s *priv);
static int cst3530_report(FAR struct cst3530_dev_s *priv,
                          FAR uint8_t *buffer);
static uint16_t cst3530_tkeys(FAR const uint8_t *records, int count);
static void cst3530_tkeys_report(FAR struct cst3530_dev_s *priv,
                                 uint16_t changed, uint16_t down);
static void cst3530_touches(FAR struct cst3530_dev_s *priv,
                            FAR const uint8_t *records, int count);
static void cst3530_worker(FAR void *arg);
static int cst3530_interrupt(int irq, FAR void *context, FAR void *arg);
static int cst3530_get(FAR struct cst3530_dev_s *priv);
static void cst3530_put(FAR struct cst3530_dev_s *priv);
static int cst3530_open(FAR struct touch_lowerhalf_s *lower);
static int cst3530_close(FAR struct touch_lowerhalf_s *lower);
static int cst3530_tkeys_open(FAR struct keyboard_lowerhalf_s *lower);
static int cst3530_tkeys_close(FAR struct keyboard_lowerhalf_s *lower);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cst3530_command
 *
 * Description:
 *   Write a register's address with no data, which is how the controller
 *   takes its commands.
 *
 ****************************************************************************/

static int cst3530_command(FAR struct cst3530_dev_s *priv, uint32_t cmd)
{
  uint8_t buffer[4];

  buffer[0] = cmd >> 24;
  buffer[1] = cmd >> 16;
  buffer[2] = cmd >> 8;
  buffer[3] = cmd;

  return i2c_write(priv->i2c, &priv->i2cconfig, buffer, sizeof(buffer));
}

/****************************************************************************
 * Name: cst3530_read
 ****************************************************************************/

static int cst3530_read(FAR struct cst3530_dev_s *priv, uint32_t reg,
                        FAR uint8_t *buffer, size_t len)
{
  int ret;

  ret = cst3530_command(priv, reg);
  if (ret >= 0)
    {
      ret = i2c_read(priv->i2c, &priv->i2cconfig, buffer, len);
    }

  return ret;
}

/****************************************************************************
 * Name: cst3530_normal
 *
 * Description:
 *   Wake the I2C interface, and put the controller in normal mode, in
 *   which it reports.
 *
 ****************************************************************************/

static int cst3530_normal(FAR struct cst3530_dev_s *priv)
{
  int ret;

  /* The first wake may not be acknowledged: it is what wakes */

  cst3530_command(priv, CST3530_CMD_WAKE);
  nxsched_usleep(CST3530_WAKE_US);

  ret = cst3530_command(priv, CST3530_CMD_WAKE);
  if (ret >= 0)
    {
      ret = cst3530_command(priv, CST3530_CMD_NORMAL1);
    }

  if (ret >= 0)
    {
      ret = cst3530_command(priv, CST3530_CMD_NORMAL2);
    }

  if (ret >= 0)
    {
      ret = cst3530_command(priv, CST3530_CMD_NORMAL3);
    }

  return ret;
}

/****************************************************************************
 * Name: cst3530_sleep
 *
 * Description:
 *   Put the controller in deep sleep, from which only a reset wakes it.
 *
 ****************************************************************************/

static void cst3530_sleep(FAR struct cst3530_dev_s *priv)
{
  cst3530_command(priv, CST3530_CMD_WAKE);
  nxsched_usleep(CST3530_WAKE_US);
  cst3530_command(priv, CST3530_CMD_WAKE);
  cst3530_command(priv, CST3530_CMD_DEEPSLEEP);
}

/****************************************************************************
 * Name: cst3530_reset
 *
 * Description:
 *   Pulse the reset line, and wait for the controller to come up.
 *
 ****************************************************************************/

static int cst3530_reset(FAR struct cst3530_dev_s *priv)
{
  int ret;

  ret = priv->config->reset(priv->config, true);
  if (ret < 0)
    {
      return ret;
    }

  nxsched_usleep(CST3530_RESET_US);

  ret = priv->config->reset(priv->config, false);
  if (ret < 0)
    {
      return ret;
    }

  nxsched_usleep(CST3530_BOOT_US);
  return OK;
}

/****************************************************************************
 * Name: cst3530_report
 *
 * Description:
 *   Read a whole report into buffer, and check it.
 *
 * Returned Value:
 *   The number of records; -EBADMSG for a report that does not add up;
 *   another negated errno value for a bus error.
 *
 ****************************************************************************/

static int cst3530_report(FAR struct cst3530_dev_s *priv,
                          FAR uint8_t *buffer)
{
  uint16_t sum;
  int nrecords;
  int ret;
  int i;

  ret = cst3530_read(priv, CST3530_REG_REPORT, buffer,
                     CST3530_HEADER_SIZE + CST3530_RECORD_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  nrecords = (buffer[3] & 0x0f) + (buffer[3] >> 4);
  if (nrecords > CST3530_MAX_RECORDS)
    {
      return -EBADMSG;
    }

  if (nrecords > 1)
    {
      ret = i2c_read(priv->i2c, &priv->i2cconfig,
                     &buffer[CST3530_HEADER_SIZE + CST3530_RECORD_SIZE],
                     (nrecords - 1) * CST3530_RECORD_SIZE);
      if (ret < 0)
        {
          return ret;
        }
    }

  sum = CST3530_CHECKSUM_SEED;
  for (i = 0; i < nrecords * CST3530_RECORD_SIZE; i++)
    {
      sum += buffer[CST3530_HEADER_SIZE + i];
    }

  if (sum != (buffer[0] | (buffer[1] << 8)))
    {
      return -EBADMSG;
    }

  return nrecords;
}

/****************************************************************************
 * Name: cst3530_tkeys
 *
 * Description:
 *   The touch keys a report has pressed, by id.
 *
 ****************************************************************************/

static uint16_t cst3530_tkeys(FAR const uint8_t *records, int count)
{
  uint16_t down = 0;
  int i;

  for (i = 0; i < count; i++)
    {
      FAR const uint8_t *record = &records[i * CST3530_RECORD_SIZE];

      if ((record[4] >> 4) != 0)
        {
          down |= 1 << (record[4] & 0x0f);
        }
    }

  return down;
}

/****************************************************************************
 * Name: cst3530_tkeys_report
 *
 * Description:
 *   A press for every touch key that went down, and a release for every
 *   one that is up again or no longer reported.  Called without the
 *   device's lock: the keyboard upper half holds its own while it opens
 *   and closes the touch keys, which takes the device's, and
 *   keyboard_event() takes it.
 *
 ****************************************************************************/

static void cst3530_tkeys_report(FAR struct cst3530_dev_s *priv,
                                 uint16_t changed, uint16_t down)
{
  FAR const struct cst3530_config_s *config = priv->config;
  int id;

  for (id = 0; changed != 0; id++, changed >>= 1)
    {
      if ((changed & 1) != 0 && config->tkeys_path != NULL &&
          id < config->ntkeys)
        {
          keyboard_event(&priv->tkeys, config->tkeys_codes[id],
                         (down & (1 << id)) != 0 ? KEYBOARD_SPECPRESS :
                                                   KEYBOARD_SPECREL);
        }
    }
}

/****************************************************************************
 * Name: cst3530_touches
 *
 * Description:
 *   Pass on the touches: first a release for every one no longer pressed,
 *   whether the report said so or left it out; then those that came, or
 *   moved, or pressed harder or lighter.  The controller goes on
 *   reporting a finger at rest, whose pressure wavers: a touch that has
 *   not moved, and whose pressure has changed by less than
 *   CONFIG_INPUT_CST3530_PRESSURE_STEP since it was last reported, is left
 *   out.  Each sample holds at most CST3530_MAX_RECORDS points, as the
 *   upper half was told.
 *
 ****************************************************************************/

static void cst3530_touches(FAR struct cst3530_dev_s *priv,
                            FAR const uint8_t *records, int count)
{
  FAR struct touch_sample_s *sample =
    (FAR struct touch_sample_s *)priv->sample;
  FAR struct touch_point_s *point;
  uint64_t timestamp = touch_get_time();
  uint16_t released;
  uint16_t down = 0;
  uint8_t pressure;
  int16_t x;
  int16_t y;
  uint8_t id;
  int i;

  for (i = 0; i < count; i++)
    {
      FAR const uint8_t *record = &records[i * CST3530_RECORD_SIZE];

      if ((record[4] >> 4) != 0)
        {
          down |= 1 << (record[4] & 0x0f);
        }
    }

  /* At most CST3530_MAX_RECORDS ids were down before */

  released = priv->down & ~down;
  if (released != 0)
    {
      memset(sample, 0, sizeof(priv->sample));
      for (id = 0; released != 0; id++, released >>= 1)
        {
          if ((released & 1) != 0)
            {
              point            = &sample->point[sample->npoints++];
              point->id        = id;
              point->x         = priv->lastx[id];
              point->y         = priv->lasty[id];
              point->timestamp = timestamp;
              point->flags     = TOUCH_UP | TOUCH_ID_VALID |
                                 TOUCH_POS_VALID;
            }
        }

      touch_event(priv->lower.priv, sample);
    }

  memset(sample, 0, sizeof(priv->sample));
  for (i = 0; i < count; i++)
    {
      FAR const uint8_t *record = &records[i * CST3530_RECORD_SIZE];
      bool held;

      if ((record[4] >> 4) == 0)
        {
          continue;
        }

      id       = record[4] & 0x0f;
      x        = record[0] | ((int16_t)(record[3] & 0x0f) << 8);
      y        = record[1] | ((int16_t)(record[3] & 0xf0) << 4);
      pressure = record[2];
      held     = (priv->down & (1 << id)) != 0;

      if (held && x == priv->lastx[id] && y == priv->lasty[id] &&
          abs(pressure - priv->lastp[id]) <
          CONFIG_INPUT_CST3530_PRESSURE_STEP)
        {
          continue;
        }

      priv->lastx[id] = x;
      priv->lasty[id] = y;
      priv->lastp[id] = pressure;

      point            = &sample->point[sample->npoints++];
      point->id        = id;
      point->x         = x;
      point->y         = y;
      point->pressure  = pressure;
      point->timestamp = timestamp;
      point->flags     = (held ? TOUCH_MOVE : TOUCH_DOWN) | TOUCH_ID_VALID |
                         TOUCH_POS_VALID | TOUCH_PRESSURE_VALID;
    }

  if (sample->npoints > 0)
    {
      touch_event(priv->lower.priv, sample);
    }

  priv->down = down;
}

/****************************************************************************
 * Name: cst3530_worker
 *
 * Description:
 *   Read a report, acknowledge it, and pass it on.
 *
 ****************************************************************************/

static void cst3530_worker(FAR void *arg)
{
  FAR struct cst3530_dev_s *priv = arg;
  uint8_t buffer[CST3530_HEADER_SIZE +
                 CST3530_MAX_RECORDS * CST3530_RECORD_SIZE];
  uint16_t tchanged = 0;
  uint16_t tdown = 0;
  int ntkeys;
  int ret;

  nxmutex_lock(&priv->lock);

  /* Closed while this was queued: the controller is asleep */

  if (priv->nopen == 0)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  /* One more try for a report that does not add up, as Hynitron's driver
   * does
   */

  ret = cst3530_report(priv, buffer);
  if (ret == -EBADMSG)
    {
      ret = cst3530_report(priv, buffer);
    }

  /* After a bus error the report is read again later, before it is
   * acknowledged, so that a release is not lost.  One that does not add
   * up twice is acknowledged and dropped, or no other would come.
   */

  if ((ret < 0 && ret != -EBADMSG) ||
      cst3530_command(priv, CST3530_CMD_ACK) < 0)
    {
      work_queue(HPWORK, &priv->work, cst3530_worker, priv, CST3530_RETRY);
      nxmutex_unlock(&priv->lock);
      return;
    }

  if (ret < 0)
    {
      iwarn("WARNING: a report garbled twice, dropped\n");
    }
  else if (buffer[2] == CST3530_REPORT_TOUCH)
    {
      /* Not a report of another type, for gestures */

      ntkeys      = buffer[3] >> 4;
      tdown       = cst3530_tkeys(&buffer[CST3530_HEADER_SIZE], ntkeys);
      tchanged    = tdown ^ priv->tdown;
      priv->tdown = tdown;

      cst3530_touches(priv, &buffer[CST3530_HEADER_SIZE +
                                    ntkeys * CST3530_RECORD_SIZE],
                      buffer[3] & 0x0f);
    }

  priv->config->enable(priv->config, true);
  nxmutex_unlock(&priv->lock);

  /* The touch keys, without the lock.  The next report's worker cannot
   * overtake with one high priority work thread (SCHED_HPNTHREADS), the
   * default, which runs one worker at a time.
   */

  cst3530_tkeys_report(priv, tchanged, tdown);
}

/****************************************************************************
 * Name: cst3530_interrupt
 *
 * Description:
 *   The interrupt is masked until the worker has read the report: the line
 *   may stay low until then.
 *
 ****************************************************************************/

static int cst3530_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cst3530_dev_s *priv = arg;
  int ret;

  priv->config->enable(priv->config, false);

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, cst3530_worker, priv, 0);
      if (ret < 0)
        {
          ierr("ERROR: queueing the worker: %d\n", ret);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: cst3530_get
 *
 * Description:
 *   Wake the controller for the first file opened, touchscreen or touch
 *   keys.
 *
 ****************************************************************************/

static int cst3530_get(FAR struct cst3530_dev_s *priv)
{
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->nopen == 0)
    {
      ret = cst3530_reset(priv);
      if (ret >= 0)
        {
          ret = cst3530_normal(priv);
        }

      if (ret < 0)
        {
          /* Held in reset, rather than awake and not reporting */

          ierr("ERROR: waking the controller: %d\n", ret);
          priv->config->reset(priv->config, true);
          nxmutex_unlock(&priv->lock);
          return ret;
        }

      priv->down  = 0;
      priv->tdown = 0;
      priv->config->enable(priv->config, true);
    }

  priv->nopen++;
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: cst3530_put
 *
 * Description:
 *   Put the controller in deep sleep when the last file is closed.
 *
 ****************************************************************************/

static void cst3530_put(FAR struct cst3530_dev_s *priv)
{
  nxmutex_lock(&priv->lock);

  if (priv->nopen > 0 && --priv->nopen == 0)
    {
      priv->config->enable(priv->config, false);
      work_cancel(HPWORK, &priv->work);
      cst3530_sleep(priv);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: cst3530_open, cst3530_close
 ****************************************************************************/

static int cst3530_open(FAR struct touch_lowerhalf_s *lower)
{
  return cst3530_get((FAR struct cst3530_dev_s *)lower);
}

static int cst3530_close(FAR struct touch_lowerhalf_s *lower)
{
  cst3530_put((FAR struct cst3530_dev_s *)lower);
  return OK;
}

/****************************************************************************
 * Name: cst3530_tkeys_open, cst3530_tkeys_close
 *
 * Description:
 *   lower->priv is the keyboard upper half's: the device is found from
 *   the lower half's place in it.
 *
 ****************************************************************************/

static int cst3530_tkeys_open(FAR struct keyboard_lowerhalf_s *lower)
{
  return cst3530_get(container_of(lower, struct cst3530_dev_s, tkeys));
}

static int cst3530_tkeys_close(FAR struct keyboard_lowerhalf_s *lower)
{
  cst3530_put(container_of(lower, struct cst3530_dev_s, tkeys));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cst3530_register(FAR struct i2c_master_s *i2c,
                     FAR const struct cst3530_config_s *config,
                     FAR const char *devpath)
{
  FAR struct cst3530_dev_s *priv;
  uint8_t info[CST3530_INFO_SIZE];
  int ret;
  int i;

  DEBUGASSERT(i2c != NULL && config != NULL && devpath != NULL &&
              config->attach != NULL && config->enable != NULL &&
              config->reset != NULL);

  if (config->tkeys_path != NULL &&
      (config->tkeys_codes == NULL || config->ntkeys > CST3530_MAX_TKEYS))
    {
      ierr("ERROR: touch keys without their codes, or too many\n");
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct cst3530_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c                 = i2c;
  priv->config              = config;
  priv->i2cconfig.frequency = config->frequency;
  priv->i2cconfig.address   = config->address;
  priv->i2cconfig.addrlen   = 7;

  nxmutex_init(&priv->lock);
  config->enable(config, false);

  /* Read the information block, which the signature tells from another
   * part's, then leave the controller asleep until a device is opened
   */

  ret = cst3530_reset(priv);
  for (i = 0; ret >= 0 && i < CST3530_INFO_TRIES; i++)
    {
      if (i > 0)
        {
          nxsched_usleep(CST3530_INFO_RETRY_US);
        }

      if (cst3530_normal(priv) >= 0 &&
          cst3530_read(priv, CST3530_REG_INFO, info, sizeof(info)) >= 0 &&
          info[2] == CST3530_INFO_MAGIC && info[3] == CST3530_INFO_MAGIC)
        {
          break;
        }
    }

  if (ret < 0 || i == CST3530_INFO_TRIES)
    {
      ierr("ERROR: no CST3530 at 0x%02x\n", config->address);
      config->reset(config, true);
      ret = ret < 0 ? ret : -ENODEV;
      goto errout;
    }

  priv->lower.xres = info[28] | ((uint16_t)info[29] << 8);
  priv->lower.yres = info[30] | ((uint16_t)info[31] << 8);

  iinfo("Chip %08" PRIx32 ", firmware %08" PRIx32 ", %ux%u, %u keys\n",
        (uint32_t)info[0] | ((uint32_t)info[1] << 8) |
        ((uint32_t)info[2] << 16) | ((uint32_t)info[3] << 24),
        (uint32_t)info[32] | ((uint32_t)info[33] << 8) |
        ((uint32_t)info[34] << 16) | ((uint32_t)info[35] << 24),
        priv->lower.xres, priv->lower.yres, info[27]);

  cst3530_sleep(priv);

  priv->lower.maxpoint = CST3530_MAX_RECORDS;
  priv->lower.flags    = config->flags;
  priv->lower.open     = cst3530_open;
  priv->lower.close    = cst3530_close;

  ret = touch_register(&priv->lower, devpath, CONFIG_INPUT_CST3530_BUFSIZE);
  if (ret < 0)
    {
      ierr("ERROR: registering %s: %d\n", devpath, ret);
      goto errout;
    }

  if (config->tkeys_path != NULL)
    {
      priv->tkeys.open  = cst3530_tkeys_open;
      priv->tkeys.close = cst3530_tkeys_close;

      ret = keyboard_register(&priv->tkeys, config->tkeys_path,
                              CONFIG_INPUT_CST3530_BUFSIZE);
      if (ret < 0)
        {
          ierr("ERROR: registering %s: %d\n", config->tkeys_path, ret);
          touch_unregister(&priv->lower, devpath);
          goto errout;
        }
    }

  /* Attached last, as the board brings its devices up before anything
   * can open them
   */

  ret = config->attach(config, cst3530_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: attaching the interrupt: %d\n", ret);
      if (config->tkeys_path != NULL)
        {
          keyboard_unregister(&priv->tkeys, config->tkeys_path);
        }

      touch_unregister(&priv->lower, devpath);
      goto errout;
    }

  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_INPUT_CST3530 */
