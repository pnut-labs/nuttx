/****************************************************************************
 * drivers/input/tca8418.c
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

/* Driver for the TI TCA8418 I2C keypad scanner.
 *
 * The controller scans the matrix itself and queues press and release
 * events in a FIFO of ten, holding its interrupt line low while any wait.
 * The interrupt hands over to a worker, which drains the FIFO, gives each
 * key its meaning through the board's keymap, and reports it to the
 * keyboard upper half.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/kbd_codec.h>
#include <nuttx/input/keyboard.h>
#include <nuttx/input/tca8418.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_INPUT_TCA8418

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define TCA8418_CFG             0x01  /* Configuration */
#define TCA8418_INT_STAT        0x02  /* Interrupt status */
#define TCA8418_KEY_LCK_EC      0x03  /* Key lock and event count */
#define TCA8418_KEY_EVENT_A     0x04  /* The FIFO's oldest event */
#define TCA8418_GPIO_INT_STAT_1 0x11  /* GPIO interrupt status, 3 bytes */
#define TCA8418_GPIO_INT_EN_1   0x1a  /* GPIO interrupt enable, 3 bytes */
#define TCA8418_KP_GPIO_1       0x1d  /* Keypad or GPIO, 3 bytes */
#define TCA8418_GPI_EM_1        0x20  /* GPI event mode, 3 bytes */
#define TCA8418_GPIO_DIR_1      0x23  /* GPIO direction, 3 bytes */
#define TCA8418_GPIO_INT_LVL_1  0x26  /* GPIO interrupt level, 3 bytes */

#define TCA8418_CFG_KE_IEN      0x01  /* Interrupt on key events */

#define TCA8418_INT_STAT_K      0x01  /* Key events; write 1 to clear */
#define TCA8418_INT_STAT_GPI    0x02  /* GPI events; write 1 to clear */

#define TCA8418_EVENT_COUNT     0x0f  /* KEY_LCK_EC: events waiting */

/* An event: bit 7 set for a press; the key, from 1, as row * 10 + column
 * + 1, whatever the matrix's size: the stride is the chip's, not the
 * board's.  Zero is an empty FIFO.
 */

#define TCA8418_EVENT_PRESS     0x80
#define TCA8418_EVENT_KEY       0x7f
#define TCA8418_KEY_STRIDE      10

/* More events than the FIFO holds in one pass is a bus gone wrong: stop,
 * rather than spin on it
 */

#define TCA8418_FIFO_DEPTH      10

#define TCA8418_NKEYS           (TCA8418_MAX_ROWS * TCA8418_MAX_COLS)

/* After a bus error the line may still be low: try again later, with the
 * interrupt masked, rather than take it again at once
 */

#define TCA8418_RETRY           MSEC2TICK(100)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A modifier, Shift or Alt */

enum tca8418_modstate_e
{
  TCA8418_MOD_OFF = 0,
  TCA8418_MOD_NEXT,                 /* Tapped: for the next key */
  TCA8418_MOD_LOCKED,               /* Tapped twice: until tapped again */
};

struct tca8418_mod_s
{
  uint8_t down;                     /* Its keys held */
  uint8_t state;                    /* enum tca8418_modstate_e */
  bool    used;                     /* A key was pressed while it was held */
  bool    shown;                    /* Reported as in effect */
};

enum
{
  TCA8418_SHIFT_MOD = 0,
  TCA8418_ALT_MOD,
  TCA8418_NMODS
};

struct tca8418_dev_s
{
  struct keyboard_lowerhalf_s lower;          /* Must be first */
  FAR struct i2c_master_s *i2c;
  FAR const struct tca8418_config_s *config;
  struct work_s work;                         /* Drains the FIFO */
  struct tca8418_mod_s mods[TCA8418_NMODS];

  /* What each key's press reported, for its release */

  uint16_t pressed[TCA8418_NKEYS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_modcode[TCA8418_NMODS] =
{
  KEYCODE_LSHIFT, KEYCODE_LALT
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tca8418_getreg, tca8418_putreg
 ****************************************************************************/

static int tca8418_getreg(FAR struct tca8418_dev_s *priv, uint8_t reg,
                          FAR uint8_t *value)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->config->frequency;
  msg[0].addr      = priv->config->address;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->frequency;
  msg[1].addr      = priv->config->address;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = value;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      ierr("ERROR: reading register 0x%02x: %d\n", reg, ret);
    }

  return ret;
}

static int tca8418_putreg(FAR struct tca8418_dev_s *priv, uint8_t reg,
                          uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  buffer[0] = reg;
  buffer[1] = value;

  msg.frequency = priv->config->frequency;
  msg.addr      = priv->config->address;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      ierr("ERROR: writing register 0x%02x: %d\n", reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: tca8418_put3
 *
 * Description:
 *   Write one of the controller's three-byte register groups.
 *
 ****************************************************************************/

static int tca8418_put3(FAR struct tca8418_dev_s *priv, uint8_t reg,
                        uint8_t b0, uint8_t b1, uint8_t b2)
{
  int ret;

  ret = tca8418_putreg(priv, reg, b0);
  if (ret >= 0)
    {
      ret = tca8418_putreg(priv, reg + 1, b1);
    }

  if (ret >= 0)
    {
      ret = tca8418_putreg(priv, reg + 2, b2);
    }

  return ret;
}

/****************************************************************************
 * Name: tca8418_configure
 *
 * Description:
 *   Scan the board's matrix, and interrupt on its key events only.  What
 *   the controller queued before is dropped.
 *
 ****************************************************************************/

static int tca8418_configure(FAR struct tca8418_dev_s *priv)
{
  FAR const struct tca8418_config_s *config = priv->config;
  uint8_t value;
  int count;
  int ret;

  /* Every pin an input, and none outside the matrix raising events: those
   * are usually unconnected and floating, and would queue phantom keys in
   * the same FIFO.  The key scanner does not go through these registers.
   */

  ret = tca8418_put3(priv, TCA8418_GPIO_DIR_1, 0, 0, 0);
  if (ret >= 0)
    {
      ret = tca8418_put3(priv, TCA8418_GPI_EM_1, 0, 0, 0);
    }

  if (ret >= 0)
    {
      ret = tca8418_put3(priv, TCA8418_GPIO_INT_LVL_1, 0, 0, 0);
    }

  if (ret >= 0)
    {
      ret = tca8418_put3(priv, TCA8418_GPIO_INT_EN_1, 0, 0, 0);
    }

  /* The matrix's pins to the scanner: rows 0 to 7, columns 0 to 7, then
   * columns 8 and 9
   */

  if (ret >= 0)
    {
      ret = tca8418_put3(priv, TCA8418_KP_GPIO_1,
                         (uint8_t)((1 << config->rows) - 1),
                         config->cols >= 8 ?
                         0xff : (uint8_t)((1 << config->cols) - 1),
                         config->cols > 8 ?
                         (uint8_t)((1 << (config->cols - 8)) - 1) : 0);
    }

  if (ret < 0)
    {
      return ret;
    }

  for (count = 0; count < TCA8418_FIFO_DEPTH; count++)
    {
      if (tca8418_getreg(priv, TCA8418_KEY_EVENT_A, &value) < 0 ||
          value == 0)
        {
          break;
        }
    }

  for (count = 0; count < 3; count++)
    {
      tca8418_getreg(priv, TCA8418_GPIO_INT_STAT_1 + count, &value);
    }

  ret = tca8418_putreg(priv, TCA8418_INT_STAT,
                       TCA8418_INT_STAT_K | TCA8418_INT_STAT_GPI);
  if (ret < 0)
    {
      return ret;
    }

  ret = tca8418_getreg(priv, TCA8418_CFG, &value);
  if (ret < 0)
    {
      return ret;
    }

  return tca8418_putreg(priv, TCA8418_CFG, value | TCA8418_CFG_KE_IEN);
}

/****************************************************************************
 * Name: tca8418_active
 ****************************************************************************/

static bool tca8418_active(FAR const struct tca8418_mod_s *mod)
{
  return mod->down > 0 || mod->state != TCA8418_MOD_OFF;
}

/****************************************************************************
 * Name: tca8418_show
 *
 * Description:
 *   Report each modifier that came into effect or went out of it.
 *
 ****************************************************************************/

static void tca8418_show(FAR struct tca8418_dev_s *priv)
{
  FAR struct tca8418_mod_s *mod;
  bool active;
  int i;

  for (i = 0; i < TCA8418_NMODS; i++)
    {
      mod    = &priv->mods[i];
      active = tca8418_active(mod);

      if (active != mod->shown)
        {
          mod->shown = active;
          keyboard_event(&priv->lower, g_modcode[i],
                         active ? KEYBOARD_SPECPRESS : KEYBOARD_SPECREL);
        }
    }
}

/****************************************************************************
 * Name: tca8418_report
 *
 * Description:
 *   Report a key: a character, or a special key, which share a range of
 *   values and differ by the event's type.
 *
 ****************************************************************************/

static void tca8418_report(FAR struct tca8418_dev_s *priv, uint16_t key,
                           bool press)
{
  if (key >= TCA8418_SPEC(0))
    {
      keyboard_event(&priv->lower, key - TCA8418_SPEC(0),
                     press ? KEYBOARD_SPECPRESS : KEYBOARD_SPECREL);
    }
  else
    {
      keyboard_event(&priv->lower, key,
                     press ? KEYBOARD_PRESS : KEYBOARD_RELEASE);
    }
}

/****************************************************************************
 * Name: tca8418_modifier
 *
 * Description:
 *   A modifier's key pressed or released.  A release with no other key
 *   pressed since its press is a tap, which steps it from off to the next
 *   key, to locked, and back to off.
 *
 ****************************************************************************/

static void tca8418_modifier(FAR struct tca8418_dev_s *priv,
                             FAR struct tca8418_mod_s *mod, bool press)
{
  if (press)
    {
      if (mod->down++ == 0)
        {
          mod->used = false;
        }
    }
  else if (mod->down > 0 && --mod->down == 0 && !mod->used)
    {
      switch (mod->state)
        {
          case TCA8418_MOD_OFF:
            mod->state = TCA8418_MOD_NEXT;
            break;

          case TCA8418_MOD_NEXT:
            mod->state = TCA8418_MOD_LOCKED;
            break;

          default:
            mod->state = TCA8418_MOD_OFF;
            break;
        }
    }

  tca8418_show(priv);
}

/****************************************************************************
 * Name: tca8418_event
 *
 * Description:
 *   Give one event from the FIFO its meaning, and report it.
 *
 ****************************************************************************/

static void tca8418_event(FAR struct tca8418_dev_s *priv, uint8_t event)
{
  FAR const struct tca8418_config_s *config = priv->config;
  FAR const uint16_t *layer;
  bool press = (event & TCA8418_EVENT_PRESS) != 0;
  uint8_t code = event & TCA8418_EVENT_KEY;
  uint16_t base;
  uint16_t key;
  uint8_t row;
  uint8_t col;
  int pos;
  int i;

  if (code == 0)
    {
      return;
    }

  code--;
  row = code / TCA8418_KEY_STRIDE;
  col = code % TCA8418_KEY_STRIDE;

  if (row >= config->rows || col >= config->cols)
    {
      iwarn("WARNING: event 0x%02x outside the %ux%u matrix\n",
            event, config->rows, config->cols);
      return;
    }

  if (config->colreverse)
    {
      col = config->cols - 1 - col;
    }

  pos  = row * config->cols + col;
  base = config->base[pos];

  if (base == TCA8418_SHIFT)
    {
      tca8418_modifier(priv, &priv->mods[TCA8418_SHIFT_MOD], press);
      return;
    }

  if (base == TCA8418_ALT)
    {
      tca8418_modifier(priv, &priv->mods[TCA8418_ALT_MOD], press);
      return;
    }

  /* A release reports what its press did */

  if (!press)
    {
      key = priv->pressed[pos];
      priv->pressed[pos] = TCA8418_NONE;
      if (key != TCA8418_NONE)
        {
          tca8418_report(priv, key, false);
        }

      return;
    }

  layer = config->base;
  if (tca8418_active(&priv->mods[TCA8418_ALT_MOD]) && config->alt != NULL)
    {
      layer = config->alt;
    }
  else if (tca8418_active(&priv->mods[TCA8418_SHIFT_MOD]) &&
           config->shift != NULL)
    {
      layer = config->shift;
    }

  key = layer[pos];
  if (key == TCA8418_NONE || key == TCA8418_SHIFT || key == TCA8418_ALT)
    {
      key = base;
    }

  /* A key with no meaning, such as a ghost of three pressed at once on a
   * matrix without diodes, leaves the modifiers as they are
   */

  priv->pressed[pos] = key;
  if (key == TCA8418_NONE)
    {
      return;
    }

  tca8418_report(priv, key, true);

  /* A modifier held went with this key; one tapped is used up by it */

  for (i = 0; i < TCA8418_NMODS; i++)
    {
      if (priv->mods[i].down > 0)
        {
          priv->mods[i].used = true;
        }

      if (priv->mods[i].state == TCA8418_MOD_NEXT)
        {
          priv->mods[i].state = TCA8418_MOD_OFF;
        }
    }

  tca8418_show(priv);
}

/****************************************************************************
 * Name: tca8418_worker
 *
 * Description:
 *   Drain the FIFO, on the high priority work queue: the I2C bus is too
 *   slow for the interrupt handler.
 *
 ****************************************************************************/

static void tca8418_worker(FAR void *arg)
{
  FAR struct tca8418_dev_s *priv = arg;
  uint8_t value;
  int count;
  int ret = OK;

  /* Until the FIFO reads empty, which also takes a key pressed meanwhile */

  for (count = 0; count < TCA8418_FIFO_DEPTH; count++)
    {
      ret = tca8418_getreg(priv, TCA8418_KEY_EVENT_A, &value);
      if (ret < 0 || value == 0)
        {
          break;
        }

      tca8418_event(priv, value);
    }

  if (ret >= 0)
    {
      ret = tca8418_putreg(priv, TCA8418_INT_STAT,
                           TCA8418_INT_STAT_K | TCA8418_INT_STAT_GPI);
    }

  if (ret >= 0)
    {
      ret = tca8418_getreg(priv, TCA8418_KEY_LCK_EC, &value);
    }

  if (ret < 0)
    {
      work_queue(HPWORK, &priv->work, tca8418_worker, priv, TCA8418_RETRY);
      return;
    }

  /* A key pressed between the last read and the acknowledgement waits with
   * the line released: come back for it, after a tick, so that a stream of
   * events cannot starve the system from this priority
   */

  if ((value & TCA8418_EVENT_COUNT) != 0)
    {
      work_queue(HPWORK, &priv->work, tca8418_worker, priv, 1);
      return;
    }

  priv->config->enable(priv->config, true);
}

/****************************************************************************
 * Name: tca8418_interrupt
 *
 * Description:
 *   The line stays low until the FIFO is read, which only the worker can
 *   do: the interrupt is masked until it has.
 *
 ****************************************************************************/

static int tca8418_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct tca8418_dev_s *priv = arg;
  int ret;

  priv->config->enable(priv->config, false);

  if (work_available(&priv->work))
    {
      ret = work_queue(HPWORK, &priv->work, tca8418_worker, priv, 0);
      if (ret < 0)
        {
          ierr("ERROR: queueing the worker: %d\n", ret);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int tca8418_register(FAR struct i2c_master_s *i2c,
                     FAR const struct tca8418_config_s *config,
                     FAR const char *devname)
{
  FAR struct tca8418_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL && devname != NULL &&
              config->base != NULL && config->attach != NULL &&
              config->enable != NULL);

  if (config->rows == 0 || config->rows > TCA8418_MAX_ROWS ||
      config->cols == 0 || config->cols > TCA8418_MAX_COLS)
    {
      ierr("ERROR: the controller scans no %ux%u matrix\n",
           config->rows, config->cols);
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct tca8418_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c    = i2c;
  priv->config = config;

  config->enable(config, false);

  ret = tca8418_configure(priv);
  if (ret < 0)
    {
      ierr("ERROR: configuring the keypad: %d\n", ret);
      goto errout;
    }

  ret = keyboard_register(&priv->lower, devname,
                          CONFIG_INPUT_TCA8418_BUFSIZE);
  if (ret < 0)
    {
      ierr("ERROR: registering %s: %d\n", devname, ret);
      goto errout;
    }

  ret = config->attach(config, tca8418_interrupt, priv);
  if (ret < 0)
    {
      ierr("ERROR: attaching the interrupt: %d\n", ret);
      keyboard_unregister(&priv->lower, devname);
      goto errout;
    }

  config->enable(config, true);
  return OK;

errout:
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_INPUT_TCA8418 */
