/****************************************************************************
 * boards/xtensa/esp32s3/lilygo-tdeck-max/src/esp32s3_tca8418.c
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

/* The keyboard: a BlackBerry Q20's, a 4 x 10 matrix scanned by a TCA8418
 * on the I2C bus.  Its interrupt is on GPIO 15, and its reset on one of
 * the XL9555's lines, released before this runs.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/kbd_codec.h>
#include <nuttx/input/tca8418.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"
#include "lilygo-tdeck-max.h"

#ifdef CONFIG_INPUT_TCA8418

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TCA8418_ADDRESS     0x34
#define TCA8418_FREQUENCY   400000

#define KBD_ROWS            4
#define KBD_COLS            10

/* Enter and Backspace are special keys, not characters: a terminal wants
 * "\n" and DEL from them, a user interface its own meanings.  Sym, right
 * of Space, is Find.
 */

#define KBD_BS              TCA8418_SPEC(KEYCODE_BACKDEL)
#define KBD_CR              TCA8418_SPEC(KEYCODE_ENTER)
#define KBD_FIND            TCA8418_SPEC(KEYCODE_FIND)
#define KBD_SH              TCA8418_SHIFT
#define KBD_ALT             TCA8418_ALT
#define KBD__               TCA8418_NONE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tdeck_kbd_attach(FAR const struct tca8418_config_s *config,
                             xcpt_t isr, FAR void *arg);
static void tdeck_kbd_enable(FAR const struct tca8418_config_s *config,
                             bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The keys as printed, left to right and top to bottom.  Alt is left of Z;
 * the bottom row has five keys: Shift, 0, Space, Sym and Shift.  The
 * matrix's columns are wired in reverse (colreverse below).
 */

static const uint16_t g_kbd_base[KBD_ROWS * KBD_COLS] =
{
  'q',     'w', 'e', 'r', 't', 'y',    'u', 'i', 'o',      'p',
  'a',     's', 'd', 'f', 'g', 'h',    'j', 'k', 'l',      KBD_BS,
  KBD_ALT, 'z', 'x', 'c', 'v', 'b',    'n', 'm', '$',      KBD_CR,
  KBD__,   KBD__, KBD__, KBD__, KBD__,
  KBD_SH,  '0', ' ', KBD_FIND, KBD_SH
};

static const uint16_t g_kbd_shift[KBD_ROWS * KBD_COLS] =
{
  'Q',     'W', 'E', 'R', 'T', 'Y',    'U', 'I', 'O',      'P',
  'A',     'S', 'D', 'F', 'G', 'H',    'J', 'K', 'L',      KBD__,
  KBD__,   'Z', 'X', 'C', 'V', 'B',    'N', 'M', KBD__,    KBD__,
  KBD__,   KBD__, KBD__, KBD__, KBD__,
  KBD__,   KBD__, KBD__, KBD__, KBD__
};

/* Alt: the digits and symbols printed on the keys */

static const uint16_t g_kbd_alt[KBD_ROWS * KBD_COLS] =
{
  '#',     '1', '2', '3', '(', ')',    '_', '-', '+',      '@',
  '*',     '4', '5', '6', '/', ':',    ';', '\'', '"',     KBD__,
  KBD__,   '7', '8', '9', '?', '!',    ',', '.', KBD__,    KBD__,
  KBD__,   KBD__, KBD__, KBD__, KBD__,
  KBD__,   KBD__, KBD__, KBD__, KBD__
};

static const struct tca8418_config_s g_kbd =
{
  .frequency  = TCA8418_FREQUENCY,
  .address    = TCA8418_ADDRESS,
  .rows       = KBD_ROWS,
  .cols       = KBD_COLS,
  .colreverse = true,
  .base       = g_kbd_base,
  .shift      = g_kbd_shift,
  .alt        = g_kbd_alt,
  .attach     = tdeck_kbd_attach,
  .enable     = tdeck_kbd_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_kbd_attach
 *
 * Description:
 *   Attach the handler, and leave the interrupt disabled until the driver
 *   enables it: esp_gpio_irq() enables it as it attaches.
 *
 ****************************************************************************/

static int tdeck_kbd_attach(FAR const struct tca8418_config_s *config,
                            xcpt_t isr, FAR void *arg)
{
  int ret;

  /* It returns 1 (-ERROR), not a negated errno, when it fails */

  ret = esp_gpio_irq(BOARD_KEYBOARD_INT, isr, arg);
  if (ret != OK)
    {
      return ret < 0 ? ret : -EIO;
    }

  esp_gpioirqdisable(BOARD_KEYBOARD_INT);
  return OK;
}

/****************************************************************************
 * Name: tdeck_kbd_enable
 ****************************************************************************/

static void tdeck_kbd_enable(FAR const struct tca8418_config_s *config,
                             bool enable)
{
  if (enable)
    {
      esp_gpioirqenable(BOARD_KEYBOARD_INT);
    }
  else
    {
      esp_gpioirqdisable(BOARD_KEYBOARD_INT);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tdeck_keyboard_initialize
 *
 * Description:
 *   Register the keyboard as /dev/kbd0.  The interrupt is taken on the low
 *   level, not the edge: the controller holds the line low until its FIFO
 *   is read, and an edge missed would leave it low for ever.
 *
 ****************************************************************************/

int tdeck_keyboard_initialize(FAR struct i2c_master_s *i2c)
{
  esp_configgpio(BOARD_KEYBOARD_INT, INPUT | PULLUP | ONLOW);

  return tca8418_register(i2c, &g_kbd, "/dev/kbd0");
}

#endif /* CONFIG_INPUT_TCA8418 */
