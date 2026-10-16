/****************************************************************************
 * drivers/input/drv2605.c
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
 * Force feedback driver for the TI DRV2605 and DRV2605L haptic drivers,
 * with an ERM motor in open loop.
 *
 * Two ways of driving the motor are used.  Real-time playback sets the
 * drive level directly and keeps it until it is changed, which is how a
 * constant effect or a rumble runs for as long as it was asked to.  The
 * built-in library plays short, shaped effects (clicks, bumps, buzzes) from
 * the chip's ROM once started, and clears its GO bit when done.  Timing on
 * the host side, both the delay before an effect and the end of it, runs
 * on the low priority work queue.
 *
 * Full strength is the chip's overdrive clamp (OD_CLAMP), left at its
 * default, 0x8c: about 3 V in open loop.
 *
 * Playback and the gain write to the chip over I2C, so they may sleep;
 * NuttX's force feedback upper half calls them with its mutex held, not
 * from interrupts.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/bits.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/ff.h>
#include <nuttx/input/drv2605.h>

#ifdef CONFIG_FF_DRV2605

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define DRV2605_REG_STATUS       0x00
#define DRV2605_REG_MODE         0x01
#define DRV2605_REG_RTPIN        0x02
#define DRV2605_REG_LIBRARY      0x03
#define DRV2605_REG_WAVESEQ1     0x04  /* ...to 0x0b */
#define DRV2605_REG_GO           0x0c
#define DRV2605_REG_ODT          0x0d  /* Overdrive time offset */
#define DRV2605_REG_SPT          0x0e  /* Sustain time offset, positive */
#define DRV2605_REG_SNT          0x0f  /* Sustain time offset, negative */
#define DRV2605_REG_BRT          0x10  /* Brake time offset */
#define DRV2605_REG_FEEDBACK     0x1a
#define DRV2605_REG_CONTROL3     0x1d

/* STATUS */

#define DRV2605_ID_SHIFT         5
#define DRV2605_ID_DRV2605       3
#define DRV2605_ID_DRV2605L      7

/* MODE */

#define DRV2605_MODE_STANDBY     (1 << 6)
#define DRV2605_MODE_INTTRIG     0x00  /* Library, started by GO */
#define DRV2605_MODE_RTP         0x05  /* Real-time playback */

/* LIBRARY */

#define DRV2605_LIBRARY_MASK     0x07

/* FEEDBACK */

#define DRV2605_FEEDBACK_LRA     (1 << 7)

/* CONTROL3 */

#define DRV2605_CONTROL3_ERM_OPEN_LOOP (1 << 5)
#define DRV2605_CONTROL3_RTP_UNSIGNED  (1 << 3)

/* Real-time playback takes a signed level; 127 is full forward drive */

#define DRV2605_RTP_MAX          127

/* How often to look whether a library effect has finished */

#define DRV2605_POLL_MS          20

/* Effects kept at once */

#define DRV2605_MAX_EFFECTS      8

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum drv2605_kind_e
{
  DRV2605_NONE = 0,          /* No effect uploaded in this slot */
  DRV2605_LEVEL,             /* Real-time playback at a level */
  DRV2605_SEQUENCE           /* Library effects */
};

enum drv2605_state_e
{
  DRV2605_IDLE = 0,          /* Nothing playing, chip in standby */
  DRV2605_DELAYED,           /* Waiting out replay.delay */
  DRV2605_RUNNING            /* Motor driven */
};

struct drv2605_effect_s
{
  uint8_t kind;                            /* enum drv2605_kind_e */
  uint8_t level;                           /* DRV2605_LEVEL, 0-127 */
  uint8_t sequence[DRV2605_SEQUENCE_LEN];  /* DRV2605_SEQUENCE */
  uint16_t length;                         /* ms; 0 runs until stopped */
  uint16_t delay;                          /* ms */
};

struct drv2605_dev_s
{
  struct ff_lowerhalf_s lower;             /* Must be first */

  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  uint32_t frequency;

  mutex_t lock;                            /* Protects the chip and state */
  struct work_s work;                      /* Delay, end and polling */
  clock_t deadline;                        /* When the work is due */
  bool armed;                              /* The work is for the effect */
  uint8_t state;                           /* enum drv2605_state_e */
  int playing;                             /* Effect id, or -1 */
  uint16_t gain;                           /* FF_GAIN, 0-0xffff */

  struct drv2605_effect_s effects[DRV2605_MAX_EFFECTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int drv2605_getreg(FAR struct drv2605_dev_s *priv, uint8_t reg,
                          FAR uint8_t *value);
static int drv2605_putreg(FAR struct drv2605_dev_s *priv, uint8_t reg,
                          uint8_t value);
static int drv2605_modifyreg(FAR struct drv2605_dev_s *priv, uint8_t reg,
                             uint8_t clearbits, uint8_t setbits);
static uint8_t drv2605_scaled(FAR struct drv2605_dev_s *priv,
                              uint8_t level);
static void drv2605_schedule(FAR struct drv2605_dev_s *priv,
                             unsigned int ms);
static int drv2605_start(FAR struct drv2605_dev_s *priv);
static void drv2605_stop(FAR struct drv2605_dev_s *priv);
static void drv2605_worker(FAR void *arg);

/* Force feedback lower half methods */

static int drv2605_upload(FAR struct ff_lowerhalf_s *lower,
                          FAR struct ff_effect *effect,
                          FAR struct ff_effect *old);
static int drv2605_erase(FAR struct ff_lowerhalf_s *lower, int effect_id);
static int drv2605_playback(FAR struct ff_lowerhalf_s *lower,
                            int effect_id, int value);
static void drv2605_set_gain(FAR struct ff_lowerhalf_s *lower,
                             uint16_t gain);
static int drv2605_control(FAR struct ff_lowerhalf_s *lower, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605_getreg
 ****************************************************************************/

static int drv2605_getreg(FAR struct drv2605_dev_s *priv, uint8_t reg,
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
      ierr("ERROR: Read of 0x%02x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2605_putreg
 ****************************************************************************/

static int drv2605_putreg(FAR struct drv2605_dev_s *priv, uint8_t reg,
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
      ierr("ERROR: Write to 0x%02x failed: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2605_modifyreg
 ****************************************************************************/

static int drv2605_modifyreg(FAR struct drv2605_dev_s *priv, uint8_t reg,
                             uint8_t clearbits, uint8_t setbits)
{
  uint8_t value;
  int ret;

  ret = drv2605_getreg(priv, reg, &value);
  if (ret < 0)
    {
      return ret;
    }

  return drv2605_putreg(priv, reg, (value & ~clearbits) | setbits);
}

/****************************************************************************
 * Name: drv2605_scaled
 *
 * Description:
 *   Apply the gain to a real-time playback level.
 *
 ****************************************************************************/

static uint8_t drv2605_scaled(FAR struct drv2605_dev_s *priv, uint8_t level)
{
  return (uint8_t)(((uint32_t)level * priv->gain + 0x7fff) / 0xffff);
}

/****************************************************************************
 * Name: drv2605_schedule
 *
 * Description:
 *   Run the worker in ms milliseconds.  The flag and the deadline are what
 *   let a worker that was already on its way when the effect was stopped,
 *   or the work was rescheduled, see that it is not for the effect playing
 *   now, and leave it alone.
 *
 ****************************************************************************/

static void drv2605_schedule(FAR struct drv2605_dev_s *priv,
                             unsigned int ms)
{
  clock_t ticks = MSEC2TICK(ms);

  priv->deadline = clock_systime_ticks() + ticks;
  priv->armed    = true;
  work_queue(LPWORK, &priv->work, drv2605_worker, priv, ticks);
}

/****************************************************************************
 * Name: drv2605_start
 *
 * Description:
 *   Start the effect in priv->playing.  Called with priv->lock held.
 *
 ****************************************************************************/

static int drv2605_start(FAR struct drv2605_dev_s *priv)
{
  FAR struct drv2605_effect_s *effect = &priv->effects[priv->playing];
  int ret;
  int i;

  priv->state = DRV2605_RUNNING;

  if (effect->kind == DRV2605_LEVEL)
    {
      /* The level goes in first, so the motor starts at it */

      ret = drv2605_putreg(priv, DRV2605_REG_RTPIN,
                           drv2605_scaled(priv, effect->level));
      if (ret >= 0)
        {
          ret = drv2605_putreg(priv, DRV2605_REG_MODE, DRV2605_MODE_RTP);
        }

      if (ret >= 0 && effect->length > 0)
        {
          drv2605_schedule(priv, effect->length);
        }
    }
  else
    {
      ret = drv2605_putreg(priv, DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);

      for (i = 0; ret >= 0 && i < DRV2605_SEQUENCE_LEN; i++)
        {
          ret = drv2605_putreg(priv, DRV2605_REG_WAVESEQ1 + i,
                               effect->sequence[i]);
          if (effect->sequence[i] == 0)
            {
              break;
            }
        }

      if (ret >= 0)
        {
          ret = drv2605_putreg(priv, DRV2605_REG_GO, 1);
        }

      if (ret >= 0)
        {
          drv2605_schedule(priv, DRV2605_POLL_MS);
        }
    }

  if (ret < 0)
    {
      drv2605_stop(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: drv2605_stop
 *
 * Description:
 *   Stop the motor and put the chip in standby.  Leaving real-time playback
 *   stops the drive at once; clearing GO cancels a library sequence.
 *   Called with priv->lock held.
 *
 ****************************************************************************/

static void drv2605_stop(FAR struct drv2605_dev_s *priv)
{
  work_cancel(LPWORK, &priv->work);
  priv->armed = false;

  if (priv->state == DRV2605_RUNNING)
    {
      drv2605_putreg(priv, DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
      drv2605_putreg(priv, DRV2605_REG_GO, 0);
      drv2605_putreg(priv, DRV2605_REG_RTPIN, 0);
    }

  drv2605_putreg(priv, DRV2605_REG_MODE,
                 DRV2605_MODE_STANDBY | DRV2605_MODE_INTTRIG);

  priv->state   = DRV2605_IDLE;
  priv->playing = -1;
}

/****************************************************************************
 * Name: drv2605_worker
 *
 * Description:
 *   Start a delayed effect, end a timed one, or look whether a library
 *   sequence has finished.
 *
 ****************************************************************************/

static void drv2605_worker(FAR void *arg)
{
  FAR struct drv2605_dev_s *priv = arg;
  uint8_t go;

  nxmutex_lock(&priv->lock);

  if (!priv->armed || !clock_compare(priv->deadline, clock_systime_ticks()))
    {
      /* Stopped or rescheduled while this was on its way: it is not for
       * the effect playing now, if any, and new work will come if needed
       */

      nxmutex_unlock(&priv->lock);
      return;
    }

  priv->armed = false;

  switch (priv->state)
    {
      case DRV2605_DELAYED:
        drv2605_start(priv);
        break;

      case DRV2605_RUNNING:
        if (priv->effects[priv->playing].kind == DRV2605_SEQUENCE &&
            drv2605_getreg(priv, DRV2605_REG_GO, &go) >= 0 &&
            (go & 1) != 0)
          {
            drv2605_schedule(priv, DRV2605_POLL_MS);
          }
        else
          {
            drv2605_stop(priv);
          }
        break;

      default:
        break;
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: drv2605_upload
 ****************************************************************************/

static int drv2605_upload(FAR struct ff_lowerhalf_s *lower,
                          FAR struct ff_effect *effect,
                          FAR struct ff_effect *old)
{
  FAR struct drv2605_dev_s *priv = (FAR struct drv2605_dev_s *)lower;
  struct drv2605_effect_s slot;
  uint32_t magnitude;
  uint32_t i;

  if (effect->id < 0 || effect->id >= DRV2605_MAX_EFFECTS)
    {
      return -EINVAL;
    }

  memset(&slot, 0, sizeof(slot));
  slot.length = effect->replay.length;
  slot.delay  = effect->replay.delay;

  switch (effect->type)
    {
      case FF_CONSTANT:
        magnitude = effect->u.constant.level < 0 ?
                    -(int32_t)effect->u.constant.level :
                    effect->u.constant.level;
        slot.kind  = DRV2605_LEVEL;
        slot.level = (magnitude * DRV2605_RTP_MAX + 0x3fff) / 0x7fff;
        break;

      case FF_RUMBLE:
        magnitude = MAX(effect->u.rumble.strong_magnitude,
                        effect->u.rumble.weak_magnitude);
        slot.kind  = DRV2605_LEVEL;
        slot.level = (magnitude * DRV2605_RTP_MAX + 0x7fff) / 0xffff;
        break;

      case FF_PERIODIC:
        if (effect->u.periodic.waveform != FF_CUSTOM ||
            effect->u.periodic.custom_len < 1 ||
            effect->u.periodic.custom_len > DRV2605_SEQUENCE_LEN ||
            effect->u.periodic.custom_data == NULL)
          {
            return -EINVAL;
          }

        for (i = 0; i < effect->u.periodic.custom_len; i++)
          {
            if (effect->u.periodic.custom_data[i] < 1 ||
                effect->u.periodic.custom_data[i] > DRV2605_LIBRARY_EFFECTS)
              {
                return -EINVAL;
              }

            slot.sequence[i] = effect->u.periodic.custom_data[i];
          }

        slot.kind = DRV2605_SEQUENCE;
        break;

      default:
        return -EINVAL;
    }

  /* An effect that is running keeps going as it was, but one still
   * waiting out its delay starts with the new parameters, and the gain is
   * applied to the new level.
   */

  nxmutex_lock(&priv->lock);
  priv->effects[effect->id] = slot;
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: drv2605_erase
 ****************************************************************************/

static int drv2605_erase(FAR struct ff_lowerhalf_s *lower, int effect_id)
{
  FAR struct drv2605_dev_s *priv = (FAR struct drv2605_dev_s *)lower;

  if (effect_id < 0 || effect_id >= DRV2605_MAX_EFFECTS)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  if (priv->playing == effect_id)
    {
      drv2605_stop(priv);
    }

  priv->effects[effect_id].kind = DRV2605_NONE;
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: drv2605_playback
 *
 * Description:
 *   Start an effect, value > 0, or stop it, value == 0.  Starting one
 *   stops whatever was playing.
 *
 ****************************************************************************/

static int drv2605_playback(FAR struct ff_lowerhalf_s *lower,
                            int effect_id, int value)
{
  FAR struct drv2605_dev_s *priv = (FAR struct drv2605_dev_s *)lower;
  int ret = OK;

  if (effect_id < 0 || effect_id >= DRV2605_MAX_EFFECTS)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  if (value == 0)
    {
      if (priv->playing == effect_id)
        {
          drv2605_stop(priv);
        }
    }
  else if (priv->effects[effect_id].kind == DRV2605_NONE)
    {
      ret = -EINVAL;
    }
  else
    {
      if (priv->playing >= 0)
        {
          drv2605_stop(priv);
        }

      priv->playing = effect_id;

      if (priv->effects[effect_id].delay > 0)
        {
          priv->state = DRV2605_DELAYED;
          drv2605_schedule(priv, priv->effects[effect_id].delay);
        }
      else
        {
          ret = drv2605_start(priv);
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: drv2605_set_gain
 ****************************************************************************/

static void drv2605_set_gain(FAR struct ff_lowerhalf_s *lower,
                             uint16_t gain)
{
  FAR struct drv2605_dev_s *priv = (FAR struct drv2605_dev_s *)lower;

  nxmutex_lock(&priv->lock);

  priv->gain = gain;

  if (priv->state == DRV2605_RUNNING &&
      priv->effects[priv->playing].kind == DRV2605_LEVEL)
    {
      drv2605_putreg(priv, DRV2605_REG_RTPIN,
                     drv2605_scaled(priv,
                                    priv->effects[priv->playing].level));
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: drv2605_control
 ****************************************************************************/

static int drv2605_control(FAR struct ff_lowerhalf_s *lower, int cmd,
                           unsigned long arg)
{
  FAR struct drv2605_dev_s *priv = (FAR struct drv2605_dev_s *)lower;
  FAR int *busy = (FAR int *)(uintptr_t)arg;

  if (cmd != DRV2605IOC_BUSY)
    {
      return -ENOTTY;
    }

  if (busy == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);
  *busy = priv->state != DRV2605_IDLE;
  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv2605_register
 *
 * Description:
 *   Configure a DRV2605 and register it as a force feedback device.  See
 *   include/nuttx/input/drv2605.h.
 *
 ****************************************************************************/

int drv2605_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr, uint32_t frequency, uint8_t library)
{
  FAR struct drv2605_dev_s *priv;
  uint8_t status = 0;
  uint8_t id;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  if (library < 1 || library > 5)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct drv2605_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->frequency = frequency;
  priv->playing   = -1;
  priv->gain      = 0xffff;

  nxmutex_init(&priv->lock);

  /* The DRV2604 variants have RAM instead of the effect library */

  ret = drv2605_getreg(priv, DRV2605_REG_STATUS, &status);
  id  = status >> DRV2605_ID_SHIFT;
  if (ret < 0 || (id != DRV2605_ID_DRV2605 && id != DRV2605_ID_DRV2605L))
    {
      ierr("ERROR: No DRV2605 at 0x%02x (%d, status 0x%02x)\n",
           addr, ret, status);
      nxmutex_destroy(&priv->lock);
      kmm_free(priv);
      return -ENODEV;
    }

  /* Out of standby while it is set up.  No overdrive, sustain or brake
   * offsets: the library's timing as it is.
   */

  ret = drv2605_putreg(priv, DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
  if (ret >= 0)
    {
      ret = drv2605_putreg(priv, DRV2605_REG_RTPIN, 0);
    }

  if (ret >= 0)
    {
      ret = drv2605_putreg(priv, DRV2605_REG_LIBRARY,
                           library & DRV2605_LIBRARY_MASK);
    }

  if (ret >= 0)
    {
      ret = drv2605_putreg(priv, DRV2605_REG_ODT, 0);
    }

  if (ret >= 0)
    {
      ret = drv2605_putreg(priv, DRV2605_REG_SPT, 0);
    }

  if (ret >= 0)
    {
      ret = drv2605_putreg(priv, DRV2605_REG_SNT, 0);
    }

  if (ret >= 0)
    {
      ret = drv2605_putreg(priv, DRV2605_REG_BRT, 0);
    }

  /* ERM, open loop, signed real-time playback data */

  if (ret >= 0)
    {
      ret = drv2605_modifyreg(priv, DRV2605_REG_FEEDBACK,
                              DRV2605_FEEDBACK_LRA, 0);
    }

  if (ret >= 0)
    {
      ret = drv2605_modifyreg(priv, DRV2605_REG_CONTROL3,
                              DRV2605_CONTROL3_RTP_UNSIGNED,
                              DRV2605_CONTROL3_ERM_OPEN_LOOP);
    }

  if (ret >= 0)
    {
      ret = drv2605_putreg(priv, DRV2605_REG_MODE,
                           DRV2605_MODE_STANDBY | DRV2605_MODE_INTTRIG);
    }

  if (ret < 0)
    {
      goto errout;
    }

  priv->lower.upload   = drv2605_upload;
  priv->lower.erase    = drv2605_erase;
  priv->lower.playback = drv2605_playback;
  priv->lower.set_gain = drv2605_set_gain;
  priv->lower.control  = drv2605_control;

  set_bit(FF_CONSTANT, priv->lower.ffbit);
  set_bit(FF_RUMBLE, priv->lower.ffbit);
  set_bit(FF_PERIODIC, priv->lower.ffbit);
  set_bit(FF_CUSTOM, priv->lower.ffbit);
  set_bit(FF_GAIN, priv->lower.ffbit);

  ret = ff_register(&priv->lower, devpath, DRV2605_MAX_EFFECTS);
  if (ret < 0)
    {
      ierr("ERROR: ff_register failed: %d\n", ret);
      goto errout;
    }

  return OK;

errout:

  /* Back to standby, if the chip still listens */

  drv2605_putreg(priv, DRV2605_REG_MODE,
                 DRV2605_MODE_STANDBY | DRV2605_MODE_INTTRIG);
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_FF_DRV2605 */
